# arbotix-sim
A simulator for the ArbotiX-M robot controller board. It runs the real
firmware on your computer, emulating the board's chip and servos, so you can
develop and test robot code without the physical hardware.

## Dear ImGui submodule

The GUI depends on Dear ImGui, included as a git submodule:

```sh
git clone --recurse-submodules <this repo's URL>
# or, if you already cloned it:
git submodule update --init
```

## Prerequisites

**Linux (Debian/Ubuntu):**

```sh
sudo apt install build-essential cmake pkg-config gcc-avr avr-libc \
    simavr libsimavr-dev libelf-dev python3-serial \
    libglfw3-dev libgl-dev
```

**macOS:**

`simavr` and `avr-gcc` aren't in Homebrew core; they're in the `osx-cross/avr` tap
after installing tap, you'll need to add trust by `brew trust osx-cross/avr`:

```sh
brew tap osx-cross/avr
brew install cmake pkg-config libelf glfw avr-gcc simavr
pip3 install pyserial
```

## Build
This produces `arbotix_gui` and the `core` library it links against.
```sh
cmake -S . -B build
cmake --build build
```

## Run

```sh
./build/arbotix_gui <firmware.elf> --link /tmp/ttyArbotiX0
# in another terminal, connect to it like a real serial port:
python3 -m serial.tools.miniterm /tmp/ttyArbotiX0 38400
```

`<firmware.elf>` is the compiled firmware you want to run. See
[`sketch/`](sketch/) for a minimal test firmware and how to build it. Other
options:

| Flag          | Default             | What it does                               |
| ------------- | ------------------- | ------------------------------------------ |
| `--link PATH` | `/tmp/ttyArbotiX0`  | Path to the virtual serial port            |
| `--ipc PATH`  | `/tmp/arbotix.sock` | Socket that streams live servo state       |
| `--servo ID`  | IDs 1 to 24         | Add a virtual servo (repeat for each one)  |
| `-v`          | off                 | Print detailed traffic logs                |

The window shows each servo's position with a slider to move it. The
simulation always runs at its own process in the background seperate from the GUI's process

Also view bash directory for run scripts.
