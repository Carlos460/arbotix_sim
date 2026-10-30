# crawler_demo

Test firmware for an 18-servo hexapod (6 legs x 3 joints), adapted from
[vanadiumlabs/arbotix](https://github.com/vanadiumlabs/arbotix). On boot it
checks the battery voltage, prints a menu, then waits for a one-key command:

| Key | Action |
| --- | --- |
| `1` | Scan all 18 servos and print their positions |
| `2` | Move to the Center pose |
| `3` | Relax all servos (torque off) |
| `4` / `5` | Left / right leg sign test |
| `6` | Check battery voltage |
| `7` | LED test |
| `8` | Walk forward (`WALK_CYCLES` gait cycles) |

## Compile

Uses the MightyCore to compile firmware:

```sh
arduino-cli compile \
  --fqbn MightyCore:avr:644:variant=modelP,pinout=standard,clock=16MHz_external,bootloader=no_bootloader \
  --export-binaries .
```

## Prerequisites

MightyCore installation

```
arduino-cli core install MightyCore:avr --additional-urls https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json
```

## Run

The scripts in [`bash/`](../../bash/) start the simulator with this setup.
