// microlib_SSD.h

#ifndef microlib_SSD1306_h
#define microlib_SSD1306_h

#include <Arduino.h>
#include <microlib.h>

// Documentation found at:
// "https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf"
// SOLOMON SYSTECH
// SEMICONDUCTOR TECHNICAL DATA (http://www.solomon-systech.com)
// SSD1306 Rev 1.1 P 1/59 Apr 2008 Copyright © 2008 Solomon Systech Limited
// Advance Information - SSD1306 - 128 x 64 Dot Matrix
// there is a copy of this document in "extras/"

#define SSD1306_ADDRESS   0x3C
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    64

/*
  The following mini library is used to write text to
  an SSD1306 OLED device using the I2C protocol.
*/

class microlib_SSD1306{

  private:

    uint8_t a;                // address
    uint8_t x, y, c;          // column, line, character
    uint8_t n; char *s;       // temp vars
    char screen[4][16];       // local screen memory (speed optim.)
    static const uint8_t fu[][8], fl[][8], lu[];  // chars definitions

    microlib_TWi *b;                         // TWi bus
    uint8_t STATUS, TWICOM[16], TWIDAT[16];  // TWi buffers
    void sendCommand(uint8_t C);             // method for TWi xfer
    void sendCommand(uint8_t C, uint8_t A);  // method for Twi xfer

  public:

    int16_t setup(microlib_TWi *bus, uint8_t address);
    int16_t putString(uint8_t X, uint8_t Y, char *S);
    int16_t updt(void);
    int16_t ready(void);
};

#endif
