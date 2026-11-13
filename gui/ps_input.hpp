#pragma once

#include <libevdev/libevdev.h>
#include <string>

std::string button_name(unsigned code);
struct libevdev *find_device(int *fd, int *rc);
