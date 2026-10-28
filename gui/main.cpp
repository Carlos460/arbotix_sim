/* gui/main.cpp: Dear ImGui front end for the ArbotiX-M simulator.
 *
 * The simulator core runs on its own background thread via sim_run().
 * This thread only drives the ImGui render loop and
 * reads a snapshot of servo state each frame.
 *
 * */

#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// Dear ImGui uses this for deprecation messages
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "simulator.h"

namespace {

struct CliOptions {
  const char *firmware_path = nullptr;
  const char *link_path = "/tmp/ttyArbotiX0";
  const char *ipc_path = "/tmp/arbotix.sock";
  uint8_t servo_ids[SIM_MAX_SERVOS] = {};
  int servo_count = 0;
  int verbose = 0;
};

void print_usage(const char *prog_name) {
  std::fprintf(stderr,
               "usage: %s <firmware.elf> [--link PATH] [--ipc PATH]"
               " [--servo ID ...] [-v]\n",
               prog_name);
}

// If argv[i] equals flag and another argument follows it, advances i past
// that argument and returns it. Otherwise returns nullptr and leaves i
// unchanged, so the caller's if/else chain falls through to the next flag.
const char *flag_value(int argc, char **argv, int &i, const char *flag) {
  if (std::strcmp(argv[i], flag) != 0 || i + 1 >= argc)
    return nullptr;
  return argv[++i];
}

bool parse_args(int argc, char **argv, CliOptions &opts) {
  for (int i = 1; i < argc; i++) {
    const char *value;

    if ((value = flag_value(argc, argv, i, "--link"))) {
      opts.link_path = value;
    } else if ((value = flag_value(argc, argv, i, "--ipc"))) {
      opts.ipc_path = value;
    } else if ((value = flag_value(argc, argv, i, "--servo"))) {
      int id = std::atoi(value);
      if (id > 0 && id < 254 && opts.servo_count < SIM_MAX_SERVOS)
        opts.servo_ids[opts.servo_count++] = static_cast<uint8_t>(id);
    } else if (!std::strcmp(argv[i], "-v")) {
      opts.verbose = 1;
    } else if (argv[i][0] != '-') {
      opts.firmware_path = argv[i];
    }
  }
  if (!opts.firmware_path)
    return false;

  if (opts.servo_count == 0) {
    for (uint8_t id = 1; id <= 5; id++)
      opts.servo_ids[opts.servo_count++] = id;
  }
  return true;
}

// Renders one row per servo: present position, an interactive goal slider,
// angle in radians, and whether it's currently moving.
// TODO: add status led light (address 25)
void draw_servo_table(const sim_snapshot_t &snap) {
  if (!ImGui::BeginTable("servos", 5,
                         ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    return;

  ImGui::TableSetupColumn("ID");
  ImGui::TableSetupColumn("Present");
  ImGui::TableSetupColumn("Goal");
  ImGui::TableSetupColumn("Angle (rad)");
  ImGui::TableSetupColumn("Moving");
  ImGui::TableHeadersRow();

  for (int i = 0; i < snap.servo_count; i++) {
    const sim_servo_snapshot_t &servo = snap.servos[i];
    ImGui::PushID(servo.id);
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::Text("%u", servo.id);

    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%u", servo.present_position);

    ImGui::TableSetColumnIndex(2);
    int goal = servo.goal_position;
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderInt("##goal", &goal, 0, 1023))
      sim_set_goal_position(servo.id, static_cast<uint16_t>(goal));

    ImGui::TableSetColumnIndex(3);
    ImGui::Text("%.3f", servo.present_radians);

    ImGui::TableSetColumnIndex(4);
    ImGui::TextUnformatted(servo.moving ? "yes" : "");

    ImGui::PopID();
  }
  ImGui::EndTable();
}

} // namespace

/* ros helper functions */

int create_socket() {
  // socket(AF_INET IPv4, SOCK_DGRAM UDP, 0 default protocal)
  return socket(AF_INET, SOCK_DGRAM, 0);
}

sockaddr_in create_dst() {
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(15000);
  dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // only local traffic
  return dst;
}

/* end ros helper functions */

int main(int argc, char **argv) {
  CliOptions opts;
  if (!parse_args(argc, argv, opts)) {
    print_usage(argv[0]);
    return 2;
  }

  sim_config_t config{};
  config.firmware_path = opts.firmware_path;
  config.link_path = opts.link_path;
  config.ipc_path = opts.ipc_path;
  config.servo_ids = opts.servo_ids;
  config.servo_count = opts.servo_count;
  config.verbose = opts.verbose;

  // seperate thread for sim
  // view public api in simulator.c
  std::thread sim_thread([&config]() { sim_run(&config); });

  if (!glfwInit()) {
    std::fprintf(stderr, "error: glfwInit failed\n");
    sim_request_stop();
    sim_thread.join();
    return 1;
  }

  const char *glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  GLFWwindow *window =
      glfwCreateWindow(720, 480, "arbotix-sim", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "error: glfwCreateWindow failed\n");
    glfwTerminate();
    sim_request_stop();
    sim_thread.join();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // ros setup
  struct command_packet {
    double angle_rad;
  };

  bool is_ros_toggle = false;
  int adapter_socket = 0;
  sockaddr_in dst;

  command_packet pck{0.5};

  // ros setup end

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    sim_snapshot_t snap;
    sim_get_snapshot(&snap);

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("arbotix-sim", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoTitleBar);

    ImGui::Text("MCU: %s", snap.mcu_name[0] ? snap.mcu_name : "(loading)");
    ImGui::Text("Host serial: %s", opts.link_path);
    ImGui::Text("IPC socket: %s (client %s)", opts.ipc_path,
                snap.ipc_client_connected ? "connected" : "none");
    ImGui::Separator();

    // ImGui both reders and handles the click here:
    // handle ros2 bridge here

    if (ImGui::Button("Connect to ROS2")) {
      is_ros_toggle = !is_ros_toggle;
      if (is_ros_toggle) {
        adapter_socket = create_socket();
        dst = create_dst();
      } else {
        close(adapter_socket);
      }
    }

    if (is_ros_toggle) {
      ImGui::Text("sending data...");
      sendto(adapter_socket, &pck, sizeof(pck), 0, (sockaddr *)&dst,
             sizeof(dst));
    }

    // end ros2 adapter
    ImGui::Separator();
    draw_servo_table(snap);

    ImGui::End();

    ImGui::Render();
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  sim_request_stop();
  sim_thread.join();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
