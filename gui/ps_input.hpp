#pragma once

enum class PsButton {
    L1,
    R1,
    DpadLeft,
    DpadDown,
    DpadRight,
    Square,
    Cross,
    Circle
};
enum class PsStick { LeftX, LeftY, RightX, RightY };

#ifdef HAVE_PS_INPUT
// reads new events and retries the connection once a second
void ps_poll();
bool ps_connected();
bool ps_pressed(PsButton button);
// stick position from -1 to 1 with up and right positive
float ps_stick(PsStick stick);
void ps_close();
#else
inline void ps_poll() {}
inline bool ps_connected() { return false; }
inline bool ps_pressed(PsButton) { return false; }
inline float ps_stick(PsStick) { return 0.0f; }
inline void ps_close() {}
#endif
