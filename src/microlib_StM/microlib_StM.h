// microlib_StM.h

/*
 * TODO:
 *
 * - sub step size is expressed in multiples of 1. Should it be
 * expressed in powers to 2 ? Since only these values are allowed:
 * 1, 2, 4, 8, 16. This could simplify some of the arithmetics and
 * some of its ambiguities (specially with negative numbers).
 *
 * - To set the power on and or off, it would be nice to set the
 * motor in a physical position that is shared between the two mode
 * (on and off) in order to avoid glitches during these transitions.
 * This glitches are small and sparse. This is fine tunning.
 *
 * - Is there a method by which we could bypass the "digitalWrite"
 * and "analogWrite" functions? and maybe increase the speed of
 * register reading and writing.
 */

#ifndef microlib_StM_h
#define microlib_StM_h

#include <Arduino.h>
#include <microlib.h>

class microlib_StM{

  private:

    static const uint8_t PWM_MSB[];

    microlib_Tmr  t; // timer

    int32_t p; // position (in substep units)
    int32_t T; // target (in substep units)
    uint8_t d; // substep size (1, 2, 4, 8, 16)
    uint8_t q; // quadrant (0, 1, 2, 3)
    uint8_t S; // Status (power, forward, backward)
    uint8_t P; // Output power state (0 or 1)

    uint8_t sa, sb;                   // substep values
    uint8_t pwa, ina1, ina2;          // Coil A power and directions
    uint8_t pwb, inb1, inb2;          // Coil B power and directions
    uint8_t a1, a2, b1, b2, pa, pb;   // tmp values

  public:

    void interval(uint32_t microseconds, uint8_t substepsize);
    void target(int32_t position);
    void shift(int32_t delta);
    void forward(void);
    void reverse(void);
    void hold(void);
    void release(void);
    int32_t getpos(void);
    int16_t updt(void);
    int16_t setup(
      uint8_t PWA, uint8_t INA1, uint8_t INA2,
      uint8_t PWB, uint8_t INB1, uint8_t INB2);
};

#endif
