# hexapod_mk2

Interbotix Hexapod Mark II firmware and it is driven by an ArbotiX Commander remote.
From [Interbotix/HexapodMKIICommander](https://github.com/Interbotix/HexapodMKIICommander/tree/master/Hexapod_Mark_II).

## Controls

In `arbotix_gui`, enable the **Commander** panel.

| Input | Action |
| --- | --- |
| Walk stick (W A S D) | Forward/back and sideways |
| Look stick, horizontal (left/right arrows) | Turn |
| `R1` | Ripple gait, smooth |
| `R2` | Amble gait, smooth (default) |
| `R3` | Ripple gait |
| `L4` | Amble gait |
| `L5` | Tripod gait |
| `L6` | Tripod gait, top speed |

## Compile

```sh
arduino-cli compile \
  --fqbn MightyCore:avr:644:variant=modelP,pinout=standard,clock=16MHz_external,bootloader=no_bootloader \
  --export-binaries .
```

See [`crawler_demo`](../crawler_demo/README.md#prerequisites) for installing MightyCore.

## Run

```sh
sh bash/hexapod_mk2.sh
```
