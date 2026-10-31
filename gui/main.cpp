/* gui/main.cpp: Dear ImGui front end for the ArbotiX-M simulator.
 *
 * The simulator core runs on its own background thread via sim_run().
 * This thread only drives the ImGui render loop and
 * reads a snapshot of servo state each frame.
 *
 * */

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

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

/* commander remote */

// ArbotiX Commander remote state. Packets go to the firmware's Serial, the
// same bytes the XBee on the real robot would deliver.
struct Commander {
  bool enabled = false;
  int walk_v = 0; // left stick
  int walk_h = 0;
  int look_v = 0; // right stick
  int look_h = 0;
  uint8_t buttons = 0;
  uint8_t clicked = 0; // buttons clicked since the last packet
  double last_send_time = 0.0;
  std::array<uint8_t, 8> last_packet{};
};

struct CommanderButton {
  const char *label;
  uint8_t bit;
};

// Bits from the ArbotiX Commander library (Commander.h).
constexpr CommanderButton kCommanderButtons[] = {
    {"R1", 0x01}, {"R2", 0x02}, {"R3", 0x04}, {"L4", 0x08},
    {"L5", 0x10}, {"L6", 0x20}, {"RT", 0x40}, {"LT", 0x80},
};

constexpr int kStickRange = 125; // Commander range
constexpr float kStickSize = 120.0f;
constexpr double kCommanderPeriod = 1.0 / 30.0;

// Maps -1-1 to a stick value.
int to_stick(float unit) {
  int value = static_cast<int>(std::lround(unit * kStickRange));
  return std::clamp(value, -kStickRange, kStickRange);
}

// Square joystick pad. Drag it with the mouse or hold the keys.
// joysticks springs back to center when released.
void draw_joystick(const char *id, int &vertical, int &horizontal, ImGuiKey up,
                   ImGuiKey down, ImGuiKey left, ImGuiKey right) {
  ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(id, ImVec2(kStickSize, kStickSize));

  if (ImGui::IsItemActive()) {
    ImVec2 mouse = ImGui::GetIO().MousePos;
    horizontal = to_stick((mouse.x - origin.x) / kStickSize * 2 - 1);
    vertical = to_stick(1 - (mouse.y - origin.y) / kStickSize * 2);
  } else {
    horizontal =
        kStickRange * (ImGui::IsKeyDown(right) - ImGui::IsKeyDown(left));
    vertical = kStickRange * (ImGui::IsKeyDown(up) - ImGui::IsKeyDown(down));
  }

  float half = kStickSize / 2;
  ImVec2 corner(origin.x + kStickSize, origin.y + kStickSize);
  ImVec2 dot(origin.x + half + horizontal * half / kStickRange,
             origin.y + half - vertical * half / kStickRange);

  ImDrawList *draw = ImGui::GetWindowDrawList();
  draw->AddRect(origin, corner, IM_COL32(150, 150, 150, 255));
  draw->AddCircleFilled(dot, 8.0f, IM_COL32(80, 160, 255, 255));
}

// Format: 0xFF, right V, right H, left V, left H, buttons, ext, checksum.
// Stick values are centered on 128.
std::array<uint8_t, 8> build_commander_packet(const Commander &cmd) {
  std::array<uint8_t, 8> packet = {
      0xFF,
      static_cast<uint8_t>(cmd.look_v + 128),
      static_cast<uint8_t>(cmd.look_h + 128),
      static_cast<uint8_t>(cmd.walk_v + 128),
      static_cast<uint8_t>(cmd.walk_h + 128),
      cmd.buttons,
      0, // ext
      0, // checksum
  };

  int sum = 0;
  for (int i = 1; i < 7; i++)
    sum += packet[i];
  packet[7] = static_cast<uint8_t>(255 - sum % 256);
  return packet;
}

void draw_commander_panel(Commander &cmd) {
  ImGui::Checkbox("Commander (send to Serial at 30 Hz)", &cmd.enabled);

  if (ImGui::BeginTable("sticks", 2)) {
    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Walk (W A S D)");
    draw_joystick("##walk", cmd.walk_v, cmd.walk_h, ImGuiKey_W, ImGuiKey_S,
                  ImGuiKey_A, ImGuiKey_D);
    ImGui::Text("V %+4d  H %+4d", cmd.walk_v, cmd.walk_h);

    ImGui::TableNextColumn();
    ImGui::TextUnformatted("Look (arrow keys)");
    draw_joystick("##look", cmd.look_v, cmd.look_h, ImGuiKey_UpArrow,
                  ImGuiKey_DownArrow, ImGuiKey_LeftArrow, ImGuiKey_RightArrow);
    ImGui::Text("V %+4d  H %+4d", cmd.look_v, cmd.look_h);

    ImGui::EndTable();
  }

  // A button is set while held. A quick click is kept until the next packet
  // so it isn't missed between sends.
  uint8_t held = 0;
  for (const CommanderButton &button : kCommanderButtons) {
    if (&button != &kCommanderButtons[0])
      ImGui::SameLine();
    if (ImGui::Button(button.label))
      cmd.clicked |= button.bit;
    if (ImGui::IsItemActive())
      held |= button.bit;
  }
  cmd.buttons = held | cmd.clicked;

  const std::array<uint8_t, 8> &p = cmd.last_packet;
  ImGui::Text("last packet: %02X %02X %02X %02X %02X %02X %02X %02X", p[0],
              p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
}

void send_commander_packet(Commander &cmd) {
  double now = ImGui::GetTime();
  if (!cmd.enabled || now - cmd.last_send_time < kCommanderPeriod)
    return;

  cmd.last_packet = build_commander_packet(cmd);
  if (sim_send_serial(cmd.last_packet.data(), cmd.last_packet.size()))
    cmd.clicked = 0;
  cmd.last_send_time = now;
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

std::vector<uint8_t> build_packet(const sim_snapshot_t &snap) {
  std::vector<uint8_t> packet;
  packet.reserve(18 * 8);

  for (int i = 0; i < snap.servo_count; i++) {
    const sim_servo_snapshot_t &servo = snap.servos[i];
    int32_t joint_id = servo.id;
    float angle = servo.present_radians;

    uint8_t buf[8];
    std::memcpy(buf, &joint_id, sizeof(int32_t));
    std::memcpy(buf + 4, &angle, sizeof(float));
    packet.insert(packet.end(), buf, buf + 8);
  }

  return packet;
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
  struct joint_packet {
    int32_t joint_id;
    float joint_position;
  };

  bool is_ros_toggle = false;
  int adapter_socket = -1;
  sockaddr_in dst;
  // ros setup end

  Commander commander;

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

    if (ImGui::Button(is_ros_toggle ? "Disconnected from ROS2"
                                    : "Connect to ROS2")) {
      is_ros_toggle = !is_ros_toggle;
      if (is_ros_toggle) {
        adapter_socket = create_socket();
        dst = create_dst();
      } else {
        close(adapter_socket);
        adapter_socket = -1;
      }
    }

    if (is_ros_toggle) {
      ImGui::Text("sending data...");
      std::vector<uint8_t> packet = build_packet(snap);

      sendto(adapter_socket, packet.data(), packet.size(), 0, (sockaddr *)&dst,
             sizeof(dst));
    }

    // end ros2 adapter

    ImGui::Separator();
    draw_commander_panel(commander);
    send_commander_packet(commander);

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
