/* PS5 DualSense controller input with libevdev */

#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <filesystem>
#include <libevdev/libevdev.h>
#include <string>
#include <unistd.h>

#include "ps_input.hpp"

namespace {

namespace fs = std::filesystem;

const std::string kDeviceName = "DualSense Wireless Controller";
const fs::path kInputDir = "/dev/input/";
constexpr float kDeadZone = 0.08f;
constexpr auto kRetryPeriod = std::chrono::seconds(1);

// evdev event that means the button is down
// the D-pad is a hat axis that reads -1 or 1
struct ButtonEvent {
    unsigned type;
    unsigned code;
    int value;
};

constexpr ButtonEvent kButtonEvents[] = {
    {EV_KEY, BTN_TL, 1},     // L1
    {EV_KEY, BTN_TR, 1},     // R1
    {EV_ABS, ABS_HAT0X, -1}, // DpadLeft
    {EV_ABS, ABS_HAT0Y, 1},  // DpadDown
    {EV_ABS, ABS_HAT0X, 1},  // DpadRight
    {EV_KEY, BTN_WEST, 1},   // Square
    {EV_KEY, BTN_SOUTH, 1},  // Cross
    {EV_KEY, BTN_EAST, 1},   // Circle
};

// same order as PsStick
constexpr unsigned kStickCodes[] = {ABS_X, ABS_Y, ABS_RX, ABS_RY};

int g_fd = -1;
libevdev *g_dev = nullptr;
std::chrono::steady_clock::time_point g_last_try;

bool is_controller(const std::string &name) {
    return name.size() >= kDeviceName.size() &&
           name.compare(name.size() - kDeviceName.size(), kDeviceName.size(),
                        kDeviceName) == 0;
}

// opens the first /dev/input/event file with the controller name
void open_device() {
    std::error_code err;
    for (const auto &entry : fs::directory_iterator(kInputDir, err)) {
        if (entry.path().filename().string().rfind("event", 0) != 0)
            continue;

        int fd = open(entry.path().c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            continue;

        libevdev *dev = nullptr;
        if (libevdev_new_from_fd(fd, &dev) == 0 &&
            is_controller(libevdev_get_name(dev))) {
            g_fd = fd;
            g_dev = dev;
            return;
        }
        libevdev_free(dev);
        close(fd);
    }
}

} // namespace

void ps_poll() {
    if (!g_dev) {
        auto now = std::chrono::steady_clock::now();
        if (now - g_last_try < kRetryPeriod)
            return;
        g_last_try = now;
        open_device();
        return;
    }

    // libevdev keeps the latest state as events are read
    // after dropped events it replays the state in sync mode
    input_event event;
    unsigned flag = LIBEVDEV_READ_FLAG_NORMAL;
    int rc;
    while ((rc = libevdev_next_event(g_dev, flag, &event)) >= 0) {
        flag = (rc == LIBEVDEV_READ_STATUS_SYNC) ? LIBEVDEV_READ_FLAG_SYNC
                                                 : LIBEVDEV_READ_FLAG_NORMAL;
    }

    if (rc == -ENODEV) // unplugged
        ps_close();
}

bool ps_connected() { return g_dev != nullptr; }

bool ps_pressed(PsButton button) {
    if (!g_dev)
        return false;
    const ButtonEvent &event = kButtonEvents[static_cast<int>(button)];
    return libevdev_get_event_value(g_dev, event.type, event.code) ==
           event.value;
}

float ps_stick(PsStick stick) {
    if (!g_dev)
        return 0.0f;

    unsigned code = kStickCodes[static_cast<int>(stick)];
    int min = libevdev_get_abs_minimum(g_dev, code);
    int max = libevdev_get_abs_maximum(g_dev, code);
    if (max <= min)
        return 0.0f;

    int value = libevdev_get_event_value(g_dev, EV_ABS, code);
    float unit = 2.0f * (value - min) / (max - min) - 1.0f;

    // evdev Y grows downward
    if (stick == PsStick::LeftY || stick == PsStick::RightY)
        unit = -unit;

    return std::fabs(unit) < kDeadZone ? 0.0f : unit;
}

void ps_close() {
    libevdev_free(g_dev);
    if (g_fd >= 0)
        close(g_fd);
    g_dev = nullptr;
    g_fd = -1;
}
