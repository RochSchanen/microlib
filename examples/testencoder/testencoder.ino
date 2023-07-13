#include "microlib.h"

// instanciate incremental encoder
microlib_IEc iec;

void setup() {

  // display
  pinMode(LED_BUILTIN, OUTPUT);

  // encoder (increment clockwise)
	pinMode(40, INPUT);     // bit 0 on port G (DT <=> signal B)
	pinMode(41, INPUT);     // bit 1 on port G (CLK <=> signal A)
  iec.setup(1, 0, PING);  // signal A: bit 1, signal B: bit 0, port: G
	iec.set(0, 0, 20);      // state: 0, minimum: 0, maximum: 20

  // one constraint is to have both pins on the same port. This
  // constraint allows to increase the code speed.

}

void loop() {
    
  // update encoder on port G  
  iec.updt(PING);

  // check cycle count parity
  if (iec.get() & 1)
    // switch on
    {digitalWrite(LED_BUILTIN, HIGH);}
  else
    // switch off
    {digitalWrite(LED_BUILTIN, LOW);};

}

/* Connect a incremental encoder between using +5V, GND, pin 40, and
 * pin 41. Compile and upload the sketch. When the incremental encoder
 * is rotated by one step (one cycle) the built-in LED should
 * alternatively turn on and off until the step count (cycle count)
 * reaches above 20 or below zero, in which case the LED will stay off,
 * and the cycle count will not change value. 
 */
