# arbotix-sim
A simulator for the ArbotiX-M board. Load and run firmware on your computer to
emulate the board's chip (simavr) and servos. Develop and test robot code without the physical hardware.

## Submodules

Dear ImGui and simavr are included as a git submodule:

```sh
git clone --recurse-submodules <this_repo_url>
# or, if you already cloned the this repo:
git submodule update --init
```

## Prerequisites

**Linux (Debian/Ubuntu):**

```sh
sudo apt install build-essential cmake pkg-config gcc-avr avr-libc \
    libelf-dev python3-serial libevdev-dev \
    libglfw3-dev libgl-dev
```

**MacOS:**

For MacOS `avr-gcc` is in the `osx-cross/avr` tap.
After installing you'll need to add trust by `brew trust osx-cross/avr`:

```sh
brew tap osx-cross/avr
brew install cmake pkg-config libelf glfw avr-gcc
pip3 install pyserial
```

## Build
This produces `arbotix_gui` and the `core` library it links against.
```sh
cmake -S . -B build
cmake --build build
```

debug options for gdb
```
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug/ -j
```

## Run

```sh
./build/arbotix_gui <firmware.elf> --link /tmp/ttyArbotiX0
# open a serial connection to communicate with the sim like a real serial port:
python3 -m serial.tools.miniterm /tmp/ttyArbotiX0 [port]
```

`<firmware.elf>` is the compiled firmware you want to run. See
[`sketch/`](sketch/) for a hexapod test firmware and how to build it.

Other options:

| Flag          | Default             | What it does                               |
| ------------- | ------------------- | ------------------------------------------ |
| `--link PATH` | `/tmp/ttyArbotiX0`  | Path to the virtual serial port            |
| `--ipc PATH`  | `/tmp/arbotix.sock` | Socket that streams live servo state       |
| `--servo ID`  | IDs 1 to 5          | Add a virtual servo (repeat for each one)  |
| `-v`          | off                 | Print detailed traffic logs                |

The window shows each servo's position with a slider to move it. The
simulation always runs at its own process in the background seperate from the GUI's process

Also view bash directory for run scripts with custom options.
