
#include <cstddef>
#include <cstdio>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <libevdev/libevdev.h>
#include <string>
#include <unistd.h>

#include "ps_input.hpp"

namespace fs = std::filesystem;

// suffix of device name
std::string target_device = "DualSense Wireless Controller";
fs::path target_directory = "/dev/input/";

// find device in /dev/input/event[n] with suffix name
struct libevdev *find_device(int *fd, int *rc) {
    std::error_code err_code;

    for (const auto &entry :
         fs::directory_iterator(target_directory, err_code)) {

        const std::string name = entry.path().filename().string();

        if (name.rfind("event", 0) != 0 || !entry.is_character_file(err_code))
            continue;

        int fd_ = open(entry.path().c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
            continue;

        struct libevdev *dev_ = nullptr;
        bool match = false;
        int rc_ = libevdev_new_from_fd(fd_, &dev_);

        if (rc_ == 0) {
            std::string dev_name = libevdev_get_name(dev_);
            match = dev_name.find(target_device, 0) != std::string::npos;
        }

        if (!match) {
            libevdev_free(dev_);
            close(fd_);
            continue;
        }

        *fd = fd_;
        *rc = rc_;
        return dev_;
    }

    if (err_code)
        std::cerr << "Directory iteration failed: " << err_code.message()
                  << "\n";
    return nullptr;
}
/* end controller helper functions */

std::string button_name(unsigned code) {
    switch (code) {
    case BTN_SOUTH:
        return "south";
    case BTN_NORTH:
        return "north";
    default:
        return NULL;
    }
}
