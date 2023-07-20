#include <Arduino.h>
#include <microlib.h>

// command codes for the SSD1306 (only a subset, See doc. in "extras/")
#define SSD1306_SETLOWCOLUMN    0x00
#define SSD1306_SETHIGHCOLUMN   0x10
#define SSD1306_SETPAGE         0xB0
#define SSD1306_SETCONTRAST     0x81
#define SSD1306_CHARGEPUMP      0x8D
#define SSD1306_SEGREMAP        0xA0
#define SSD1306_DISPLAYOFF      0xAE
#define SSD1306_DISPLAYON       0xAF
#define SSD1306_COMSCANINC      0xC0
#define SSD1306_COMSCANDEC      0xC8

// status codes for the updt() method
#define SSD1306_READY     0     // nothing to do
#define SSD1306_5         5     // check for redondancy
#define SSD1306_10       10     // Set upper page and reset column
#define SSD1306_20       20     // Send character upper data bytes
#define SSD1306_30       30     // Set lower page and reset column
#define SSD1306_40       40     // Send character lower data bytes
#define SSD1306_50       50     // select next character to print

// single byte commands (blocking)
void microlib_SSD1306::sendCommand(uint8_t C){
  uint8_t BUS_COM[2] = {TW_WRITE, TW_WRITE | TW_STOP};
  uint8_t BUS_DAT[2]; BUS_DAT[0] = 0x00; BUS_DAT[1] = C;
  b->xfer(a, BUS_DAT, BUS_COM);
  b->wait(BUS_COM);
  return;
}

// two bytes commands (blocking)
void  microlib_SSD1306::sendCommand(uint8_t C, uint8_t A){
  uint8_t BUS_COM[3] = {TW_WRITE, TW_WRITE, TW_WRITE | TW_STOP};
  uint8_t BUS_DAT[3]; BUS_DAT[0] = 0x00; BUS_DAT[1] = C; BUS_DAT[2] = A;
  b->xfer(a, BUS_DAT, BUS_COM);
  b->wait(BUS_COM);
  return;
}

// setup and start display
int16_t microlib_SSD1306::setup(microlib_TWi *bus, uint8_t address){
  x = 0; y= 0;
  b = bus; a = address;
  STATUS = SSD1306_READY;
  TWICOM[0] = TW_READY;
  sendCommand(SSD1306_DISPLAYOFF);
  sendCommand(SSD1306_SEGREMAP | 0x01);
  sendCommand(SSD1306_COMSCANDEC);
  sendCommand(SSD1306_CHARGEPUMP, 0x14);
  sendCommand(SSD1306_SETCONTRAST, 0x20);
  sendCommand(SSD1306_DISPLAYON);
  return 0;
}

// updt() method using a state machine technique
// This reduces delays in the arduino main loop
int16_t microlib_SSD1306::updt(void){

  uint8_t i, Low, High;

  if (TWICOM[0] & TW_READY){

    switch(STATUS){

      case SSD1306_READY:
        break; // do nothing

      // speed optimisation (check if character is already printed)
      case SSD1306_5:
        // characters are two bytes in height and 8 bits in width
        if (screen[y>>1][x>>3]==s[n]) STATUS = SSD1306_50; // skip
        // next state
        else STATUS = SSD1306_10;
        break;

      // Set upper page and reset column
      case SSD1306_10:
        Low  = (x & 0x0F);
        High = (x & 0xF0) >> 4;
        TWIDAT[0] = 0x00; TWIDAT[1] = SSD1306_SETPAGE  | y;
        TWIDAT[2] = 0x00; TWIDAT[3] = SSD1306_SETHIGHCOLUMN | High;
        TWIDAT[4] = 0x00; TWIDAT[5] = SSD1306_SETLOWCOLUMN  | Low;
        for(i=0; i<6; i++) TWICOM[i] = TW_WRITE;
        TWICOM[5] |= TW_STOP;
        b->xfer(a, TWIDAT, TWICOM);
        STATUS = SSD1306_20;
        break;

      // Send character upper data bytes
      case SSD1306_20:
        TWIDAT[0] = 0x40; for(i=0; i<8; i++)
          TWIDAT[i+1] = pgm_read_byte(&(fu[c][i]));
        for(i=0; i<9; i++) TWICOM[i] = TW_WRITE;
        TWICOM[8] |= TW_STOP;
        b->xfer(a, TWIDAT, TWICOM);
        STATUS = SSD1306_30;
        break;

      // Set lower page and reset column
      case SSD1306_30:
        Low  = (x & 0x0F);
        High = (x & 0xF0) >> 4;
        TWIDAT[0] = 0x00; TWIDAT[1] = SSD1306_SETPAGE  | y+1;
        TWIDAT[2] = 0x00; TWIDAT[3] = SSD1306_SETHIGHCOLUMN | High;
        TWIDAT[4] = 0x00; TWIDAT[5] = SSD1306_SETLOWCOLUMN  | Low;
        for(i=0; i<6; i++) TWICOM[i] = TW_WRITE;
        TWICOM[5] |= TW_STOP;
        b->xfer(a, TWIDAT, TWICOM);
        STATUS = SSD1306_40;
        break;

      // Send character lower data bytes
      case SSD1306_40:
        TWIDAT[0] = 0x40; for(i=0; i<8; i++)
          TWIDAT[i+1] = pgm_read_byte(&(fl[c][i]));
        for(i=0; i<9; i++) TWICOM[i] = TW_WRITE;
        TWICOM[8] |= TW_STOP;
        b->xfer(a, TWIDAT, TWICOM);
        STATUS = SSD1306_50;
        break;

      // select next character to print
      case SSD1306_50:
        n++;
        if(s[n]){ // not end-of-string
          // next column position
          x+= 8;
          // get pointer to character definition from look-up table
          c = pgm_read_byte(&(lu[(uint8_t)(s[n])-32]));
          // continue printing
          STATUS = SSD1306_5;}
        else{ // end-of-string
          s[0]='\0'; // clear string buffer
          // clear device state
          STATUS = SSD1306_READY;}
        break;
    }
  }
  return 0;
}

int16_t microlib_SSD1306::putString(uint8_t X, uint8_t Y, char *S){
  // position is 16 X 4 for characters' size of 8*16 pixels
  n = 0; s = S; y = Y<<1; x = X<<3;
  // character index from look-up table
  c = pgm_read_byte(&(lu[(uint8_t)(s[n])-32]));
  // n += 1;
  STATUS = SSD1306_10; // replace with SSD1306_05 ?
  return 0;
}

int16_t microlib_SSD1306::ready(void){
  return STATUS == SSD1306_READY;
}

// font definition upper table
const uint8_t microlib_SSD1306::fu[][8] PROGMEM = {
        {0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xff},
        {0xe0, 0x18, 0x04, 0x82, 0x02, 0x11, 0x39, 0x7d},
        {0x11, 0x11, 0x22, 0xc2, 0x04, 0x18, 0xe0, 0x00},
        {0xe0, 0x18, 0x04, 0x42, 0x12, 0x01, 0x09, 0x01},
        {0x09, 0x01, 0x12, 0x42, 0x04, 0x18, 0xe0, 0x00},
        {0xe0, 0x18, 0x04, 0x02, 0x02, 0xf1, 0x09, 0xc9},
        {0x69, 0xf1, 0x02, 0x02, 0x04, 0x18, 0xe0, 0x00},
        {0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xc0, 0xc0, 0xc0},
        {0xc0, 0xc0, 0xc0, 0xf8, 0xf0, 0xe0, 0xc0, 0x80},
        {0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0xe0, 0xe0, 0x00, 0x00, 0xe0, 0xe0, 0x00},
        {0x00, 0x00, 0xf0, 0xf8, 0x08, 0x68, 0xf8, 0xf0},
        {0x00, 0x20, 0x20, 0x30, 0xf8, 0xf8, 0x00, 0x00},
        {0x00, 0x30, 0x38, 0x08, 0x88, 0xf8, 0x70, 0x00},
        {0x00, 0x30, 0x38, 0x88, 0x88, 0xf8, 0x70, 0x00},
        {0x00, 0x00, 0xf8, 0xf8, 0x00, 0xe0, 0xe0, 0x00},
        {0x00, 0xf8, 0xf8, 0x88, 0x88, 0x88, 0x08, 0x00},
        {0x00, 0xc0, 0xe0, 0x78, 0x58, 0xc8, 0x80, 0x00},
        {0x00, 0x08, 0x08, 0x88, 0xe8, 0x78, 0x18, 0x00},
        {0x00, 0x70, 0xf8, 0xc8, 0x88, 0xf8, 0x70, 0x00},
        {0x00, 0xf0, 0xf8, 0x08, 0x08, 0xf8, 0xf0, 0x00},
        {0x80, 0x60, 0x90, 0x90, 0x60, 0x80, 0x00, 0x00},
        {0x00, 0x00, 0x80, 0x60, 0x90, 0x90, 0x60, 0x80},
        {0x00, 0x60, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x60, 0x90, 0x90, 0x60, 0x00},
        {0x00, 0xe0, 0xe0, 0x20, 0xe0, 0x20, 0xe0, 0xc0},
        {0x00, 0xe0, 0xe0, 0x20, 0x20, 0xe0, 0xc0, 0x00},
        {0x00, 0xe0, 0xe0, 0x80, 0x40, 0x60, 0x60, 0x00}};

// font definition lower table
const uint8_t microlib_SSD1306::fl[][8] PROGMEM = {
        {0x7f, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7f},
        {0x03, 0x0c, 0x10, 0x21, 0x26, 0x44, 0x48, 0x48},
        {0x48, 0x44, 0x26, 0x21, 0x10, 0x0c, 0x03, 0x00},
        {0x03, 0x0c, 0x10, 0x21, 0x24, 0x40, 0x48, 0x40},
        {0x48, 0x40, 0x24, 0x21, 0x10, 0x0c, 0x03, 0x00},
        {0x03, 0x0c, 0x10, 0x20, 0x20, 0x47, 0x4b, 0x49},
        {0x48, 0x47, 0x20, 0x20, 0x10, 0x0c, 0x03, 0x00},
        {0x00, 0x01, 0x03, 0x07, 0x0f, 0x01, 0x01, 0x01},
        {0x01, 0x01, 0x01, 0x0f, 0x07, 0x03, 0x01, 0x00},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x00, 0x00},
        {0x40, 0x7f, 0x3f, 0x08, 0x08, 0x07, 0x0f, 0x08},
        {0x00, 0x00, 0x07, 0x0f, 0x0b, 0x08, 0x0f, 0x07},
        {0x00, 0x00, 0x00, 0x00, 0x0f, 0x0f, 0x00, 0x00},
        {0x00, 0x0c, 0x0e, 0x0b, 0x09, 0x08, 0x08, 0x00},
        {0x00, 0x06, 0x0e, 0x08, 0x08, 0x0f, 0x07, 0x00},
        {0x00, 0x03, 0x03, 0x02, 0x02, 0x0f, 0x0f, 0x02},
        {0x00, 0x08, 0x08, 0x08, 0x0c, 0x07, 0x03, 0x00},
        {0x00, 0x07, 0x0f, 0x08, 0x08, 0x0f, 0x07, 0x00},
        {0x00, 0x00, 0x0e, 0x0f, 0x01, 0x00, 0x00, 0x00},
        {0x00, 0x07, 0x0f, 0x08, 0x09, 0x0f, 0x07, 0x00},
        {0x00, 0x00, 0x09, 0x0d, 0x0f, 0x03, 0x01, 0x00},
        {0x07, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x12},
        {0x12, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x07},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x12},
        {0x12, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        {0x00, 0x0f, 0x0f, 0x00, 0x07, 0x00, 0x0f, 0x0f},
        {0x00, 0x7f, 0x7f, 0x08, 0x08, 0x0f, 0x07, 0x00},
        {0x00, 0x0f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00}};

// look-up table for character definitions
// this allows to limits the size of the definitions
// table to the characters that are actually used in
// the display. there is here a maximum of 96 chars.
// undefined chars points to 0x00: the empty square glyph.
// table starts at ascii code 32 (space char). The tables
// are build using the python code "8x15_ConvertGlyphToCppTables.py"
// which reads the "8x15_.txt" human readable definition file.
// only those glyph that are explicitely defined in this text file
// are entered into the table definition.
const uint8_t microlib_SSD1306::lu[] PROGMEM = {
        0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x05, 0x06, 0x00, 0x00, 0x00, 0x09, 0x0b, 0x00,
        0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,
        0x15, 0x16, 0x00, 0x00, 0x07, 0x00, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x18,
        0x19, 0x1a, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x00,
        0x1c, 0x00, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x0c, 0x00};
