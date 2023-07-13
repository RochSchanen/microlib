// microlib_IEc.h

#ifndef microlib_IEc_h
#define microlib_IEc_h

#include "Arduino.h"

class microlib_IEc{

  private:
    
    static const uint8_t Q[5][4];
    static const int8_t  C[5][4];
    uint8_t a, b, q;
    uint8_t i1, i2, i;
    int16_t c, c1, c2;

  public:

    int16_t setup(uint8_t A, uint8_t B, uint8_t D);
    int16_t updt(uint8_t D);
    int16_t get(void);
    int16_t set(int16_t value);
    int16_t set(int16_t value, int16_t min, int16_t max);

};

#endif
