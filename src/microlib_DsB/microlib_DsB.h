// microlib_DSB.h

#ifndef microlib_DSB_h
#define microlib_DSB_h

#include <Arduino.h>
#include <microlib.h>

/* This is a convoluted way of writing text to the SSD1306 display that
 * prevents delaying the main loop. You setup a buffer to be displayed
 * on the SSD1306. The text to display is written to a buffer string.
 * Regular calls to the updt() method will go through the buffer string
 * character by character and send them to the display one by one.
 * To update the text to display, simply re-write in the buffer string
 * (do NOT forget the end-of-string char).
 */

// maximum buffer length
// (the SSD1306 display has a maximum of 16 columns with 8 pixels characters width)
#define DSB_BUFLEN 16 

class microlib_DSB{

    private:

        microlib_SSD1306 *d;           // SSD1306 display
        uint8_t x, y;                  // buffer position (fixed)
        static char b[DSB_BUFLEN + 1]; // transfer buffer (must also include the end-of-string char)
        static uint8_t n;              // stack size of diplay buffers: common to all instances
        static uint8_t i;              // the current display buffer id: common to all instances
        uint8_t j, k;                  // j is the current instance identifier, k is a temp var
        char *s;                       // s is the current instance buffer string to display

    public:

        void setup(microlib_SSD1306 *display, char *strbuf, uint8_t X, uint8_t Y);
        void updt(void);
};

#endif
