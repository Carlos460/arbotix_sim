#ifndef POSES
#define POSES

#include <avr/pgmspace.h>

// prog_uint16_t was just a typedef for uint16_t, removed in avr-libc 2.x.
// Modern avr-gcc also requires PROGMEM data to be const.
const PROGMEM uint16_t Center[] = {18,  512, 512, 312, 712, 812, 212,
                                   512, 512, 312, 712, 812, 212, 512,
                                   512, 312, 712, 812, 212};

#endif
