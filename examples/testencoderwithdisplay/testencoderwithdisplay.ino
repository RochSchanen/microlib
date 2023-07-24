// testmicrolib.ino

#include <microlib.h>

/////////////////////////////
//// INCREMENTAL ENCODER ////
/////////////////////////////

#define IE_A 40 // A: PIN 40 (port G bit 0)
#define IE_B 41 // B: PIN 41 (port G bit 1)
#define IE_I 14 // I: PIN 14 (port J bit 0)

// Incremental Encoder instance
microlib_IEc ie; // uses signal (A) and (B)

// Integrated button on the incremental encoder
microlib_Btn btn_ie; // uses signal (I)

int16_t ie_v; // encoder recorded value [steps]

/////////////////
//// DISPLAY ////
/////////////////

// Solomon Systech Diplay
microlib_SSD1306 dsp = microlib_SSD1306();

// bus
microlib_TWi bus = microlib_TWi();

// clear function (write space chars one by one, blocking)
void clear(void){
  char clrstr[]={' ','\0'};
  for(uint8_t Y=0; Y<4; Y++)
    for(uint8_t X=0; X<16; X++){
      clrstr[0]=' ';
      dsp.putString(X, Y, clrstr);
      while(!dsp.ready()){
        bus.updt();
        dsp.updt();
      }
    }
  return;
}

// string buffer (includes the end-of-string char)
char db_ie_s[] = "+@@";

// display buffer instance for printing the (ie_v) value
microlib_DSB db_ie; // uses the string buffer (dsb_ie_s)

///////////////
//// SETUP ////
///////////////

void setup() {

  // setup display ssd1306
  dsp.setup(&bus, 0x3C);
  clear();

  // setup display buffer
  db_ie.setup(&dsp, db_ie_s, 0, 0);

  // setup incremental encoder
  pinMode(IE_A, INPUT_PULLUP); // setup pin mode (todo: check "INPUT")
  pinMode(IE_B, INPUT_PULLUP); // setup pin mode
  ie.setup(0, 1, PING);        // setup encoder bits and register
  ie.set(0, -10, +10);         // setup start value

  // setup button
  btn_ie.setup(IE_I);

  return;
}

int16_t v;
int16_t n = 10;

//////////////////
// START-OF-LOOP

void loop(){

  ////////// UPDATE DEVICES STATES //////////

  // display
  bus.updt();   // update bus
  dsp.updt();   // update display
  db_ie.updt(); // update display buffer

  // incremental encoder with button
  btn_ie.updt();
  ie.updt(PING);

  ////////// UPDATE STATE VARIABLES //////////

  // update encoder value
  v = ie.get();                         // get the up-to-date value
  if (ie_v != v){                       // compare with the recorded value
    ie_v = v;                           // update the state variable
    dtostrf((float)v, 3, 0, db_ie_s);   // and the display
  }

  // reset encoder value on a button press event
  if (btn_ie.pressed()){
    ie.set(0);                          // force reset
  }

}

// END-OF-LOOP
///////////////
