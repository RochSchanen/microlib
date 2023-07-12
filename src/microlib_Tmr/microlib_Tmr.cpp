#include "Arduino.h"
#include "microlib.h"

int16_t microlib_Tmr::setup(uint32_t delay){
	d = delay;
	return 0;
}

int16_t microlib_Tmr::reset(void){
	l = micros();
	return 0;
}

int16_t microlib_Tmr::ready(void){
	uint32_t c = micros();
	return c-l > d;
}
