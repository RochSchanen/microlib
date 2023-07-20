#include <Arduino.h>
#include <microlib.h>

char    microlib_DSB::b[DSB_BUFLEN + 1];
uint8_t microlib_DSB::i = 0;
uint8_t microlib_DSB::n = 0;

void microlib_DSB::setup(microlib_SSD1306 *display, char *strbuf, uint8_t X, uint8_t Y){
    d = display;    // display device
    s = strbuf;     // local string buffer
    x = X; y = Y;   // string position
    j = n;          // set local id value
    n++;            // increase stack size
    return;
}

// when a new class instance is created, it is given a unique id.
// the ids are incremented by one with each new instanciation of the class
// an index is run cyclicly through all the ids. when a buffer is not
// empty and the display is ready for printing, the buffer string is
// copied to the transfer buffer and the buffer string is cleared and
// the buffer id is set to test the next display buffer. While a buffer
// is written, the main loop can modify other existing buffers without
// clogging the display with outdated text. To modify the buffer display,
// you only need to fill the display buffer string with the new text.

void microlib_DSB::updt(void){
    // current id (i) tested
    if(j == i){ // "j" is this instance id, compare with running id "i"
        if(s[0] > 0){ // check is the buffer is not empty
            if(d->ready()){ // check is the display is ready for xfer
                // COPY DISPLAY BUFFER STRING TO XFER BUFFER STRING
                k = 0;
                while(s[k] > 0) b[k] = s[k++];
                b[k] = s[k]; // don't forget the end-of-string char
                // CLEAR DISPLAY BUFFER STRING
                s[0] = '\0';
                // SET STRING DATA FOR DISPLAY
                d->putString(x, y, b);
                // increment "running" id
                i++;
                // cycle through the display buffer stack
                if(i == n) i = 0; // could have used a modulo operator
            }
        }else{ // tested buffer is empty, check the next one
            // increment "running" id
            i++;
            // cycle through the display buffer stack
            if(i == n) i = 0;
        }
    }
    // done
    return;
}
