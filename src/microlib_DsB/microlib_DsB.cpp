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

// When a new class instance is created, it is given a unique id (j).
// The ids are incremented by one with each new instanciation.
// An index (i) is run cyclicly through all the ids (j). When a buffer
// is not empty and the display is ready for printing, the buffer string
// is copied to the transfer buffer, the buffer string is cleared, and
// the buffer index (i) is set to test the next display buffer. While a
// buffer is written, the main loop can easily modify any buffer string
// WITHOUT CLOGGING the display with outdated text. To modify the display,
// you only need to fill the appropriate buffer string with some new text.

void microlib_DSB::updt(void){
    // compare local id (j) to the running index value (i)
    if(j == i){ // matching -> check if the buffer needs an update
        if(s[0] > 0){ // check is the buffer is not empty
            if(d->ready()){ // check is the display is ready for a transfer
                // COPY DISPLAY BUFFER STRING TO TRANSFER BUFFER STRING
                k = 0;
                while(s[k] > 0){
                    b[k] = s[k];
                    k++;}
                b[k] = s[k]; // don't forget the end-of-string char
                // CLEAR DISPLAY BUFFER STRING
                s[0] = '\0';
                // SET STRING DATA FOR DISPLAY
                d->putString(x, y, b);
                // increment "running" id
                i++;
                // cycle through the display buffer stack
                if(i == n) i = 0; // use modulo operator instead?
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
