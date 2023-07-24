// testmicrolib.ino

#include <microlib.h>

// Solomon Systech Diplay
microlib_SSD1306 dsp = microlib_SSD1306();

// bus
microlib_TWi bus = microlib_TWi();

// clear function
void clear(void){
    char clrstr[]={' ','\0'};
    for(uint8_t Y=0; Y<4; Y++)
        for(uint8_t X=0; X<16; X++){
        clrstr[0]=' '; dsp.putString(X, Y, clrstr);
        while(!dsp.ready()){bus.updt(); dsp.updt();}
        }
    return;}

void setup() {

    // setup display ssd1306
    dsp.setup(&bus, 0x3C);
    // clear display
    clear();
    // string buffer for char transfer
    char s[32];
    // fill string with data
    strcpy(s,"0123");
    // send string for printing at coordinates 0, 0
    dsp.putString(0, 0, s);
    // call to update functions until printing is done
    while(!dsp.ready()){
        // call to bus update
        bus.updt();
        // call to display update
        dsp.updt();
        // repeat
        }
    // done
    return;
}

void loop(){
    // no operations
}
