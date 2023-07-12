#include "microlib.h"

microlib_Btn b;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  b.setup(2);
}

void loop() {
  b.updt();
  if (b.pressed()) digitalWrite(LED_BUILTIN, HIGH);
  if (b.released()) digitalWrite(LED_BUILTIN, LOW);
}

/* Connect a switch between the ground and pin 2.
 * Compile and upload the sketch.
 * When the switch makes contact, the built-in LED lights up.
 * When the switch is released, the built-in LED turns off.
 */
