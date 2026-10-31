// Receives joint positions from the ArbotiX simulator over UDP and publishes
// them for robot_state_publisher / RViz and for the Gazebo joint controllers.
//
//   struct joint_packet {
//     int32_t joint_id;        // servo ID, 1..18
//     float joint_position;    // radians, 0 = servo center (tick 512)
//   };
//
// Publishes:
//   /joint_states                   sensor_msgs/JointState (all 18 joints)
//   /hexapod/<joint_name>/cmd_pos   std_msgs/Float64 (one per joint, for Gazebo)

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

struct joint_packet {
  int32_t joint_id;
  float joint_position;
};
static_assert(sizeof(joint_packet) == 8, "joint_packet must be 8 bytes");

// Servo ID -> URDF joint name. Index 0 is servo ID 1. Default is the standard
// PhantomX ArbotiX ID layout.
static const std::vector<std::string> kDefaultJointNames = {
  "j_c1_lf",    "j_c1_rf",    // 1, 2
  "j_thigh_lf", "j_thigh_rf", // 3, 4
  "j_tibia_lf", "j_tibia_rf", // 5, 6
  "j_c1_lr",    "j_c1_rr",    // 7, 8
  "j_thigh_lr", "j_thigh_rr", // 9, 10
  "j_tibia_lr", "j_tibia_rr", // 11, 12
  "j_c1_lm",    "j_c1_rm",    // 13, 14
  "j_thigh_lm", "j_thigh_rm", // 15, 16
  "j_tibia_lm", "j_tibia_rm", // 17, 18
};

class UdpJointBridge : public rclcpp::Node
{
public:
  UdpJointBridge()
  : Node("udp_joint_bridge")
  {
    // 0.0.0.0 so packets forwarded into a Docker container.
    const auto bind_address = declare_parameter<std::string>("bind_address", "0.0.0.0");
    const auto port = declare_parameter<int>("port", 15000);
    const auto rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    const auto topic_prefix = declare_parameter<std::string>("cmd_topic_prefix", "/hexapod");
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", kDefaultJointNames);
    // Use -1.0 to flip a joint that turns the wrong way.
    signs_ = declare_parameter<std::vector<double>>(
      "joint_signs", std::vector<double>(joint_names_.size(), 1.0));

    if (signs_.size() != joint_names_.size()) {
      throw std::invalid_argument("joint_signs must have one entry per joint_names entry");
    }
    // Radians added after the sign, for joints whose URDF zero isn't servo center.
    offsets_ = declare_parameter<std::vector<double>>(
      "joint_offsets", std::vector<double>(joint_names_.size(), 0.0));
    if (offsets_.size() != joint_names_.size()) {
      throw std::invalid_argument("joint_offsets must have one entry per joint_names entry");
    }
    positions_.assign(joint_names_.size(), 0.0);

    open_socket(bind_address, port);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    for (const auto & name : joint_names_) {
      cmd_pubs_.push_back(
        create_publisher<std_msgs::msg::Float64>(topic_prefix + "/" + name + "/cmd_pos", 10));
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {
        receive_packets();
        publish();
      });

    RCLCPP_INFO(
      get_logger(), "listening for joint packets on udp %s:%ld", bind_address.c_str(), port);
  }

  ~UdpJointBridge() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void open_socket(const std::string & bind_address, int port)
  {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
    }
    // Non blocking so the timer can drain whatever has arrived and move on.
    fcntl(socket_fd_, F_SETFL, fcntl(socket_fd_, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
      throw std::invalid_argument("bad bind_address: " + bind_address);
    }
    if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error(std::string("bind: ") + std::strerror(errno));
    }
  }

  void receive_packets()
  {
    joint_packet buf[64];
    while (true) {
      ssize_t n = recv(socket_fd_, buf, sizeof(buf), 0);
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(get_logger(), "recv: %s", std::strerror(errno));
        }
        return;
      }
      if (n % sizeof(joint_packet) != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "dropped %zd byte datagram (not a multiple of %zu)",
          n, sizeof(joint_packet));
        continue;
      }
      for (size_t i = 0; i < n / sizeof(joint_packet); i++) {
        const int32_t id = buf[i].joint_id;
        if (id < 1 || static_cast<size_t>(id) > joint_names_.size()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "unknown joint_id %d", id);
          continue;
        }
        positions_[id - 1] = signs_[id - 1] * buf[i].joint_position + offsets_[id - 1];
      }
    }
  }

  void publish()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = positions_;
    joint_state_pub_->publish(msg);

    for (size_t i = 0; i < cmd_pubs_.size(); i++) {
      std_msgs::msg::Float64 cmd;
      cmd.data = positions_[i];
      cmd_pubs_[i]->publish(cmd);
    }
  }

  int socket_fd_ = -1;
  std::vector<std::string> joint_names_;
  std::vector<double> signs_;
  std::vector<double> offsets_;
  std::vector<double> positions_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> cmd_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpJointBridge>());
  rclcpp::shutdown();
  return 0;
}
