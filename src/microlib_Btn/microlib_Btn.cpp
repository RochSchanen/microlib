#include "Arduino.h"
#include "microlib.h"

int16_t microlib_Btn::setup(uint8_t pin){
	p = pin; pinMode(p, INPUT_PULLUP);
	s = digitalRead(p); q = 0; r = 0;
	t.setup(100000); // 100ms
	t.reset();
	return 0;
}

int16_t microlib_Btn::updt(void){
	// wait for timer
	if (t.ready()){
		i = digitalRead(p);
		// change of pin state?
		if (i == s) return 0;
		switch(i){
			case 0: // button pressed
				q = 1;
				break;
			case 1: // button released
				r = 1;
				break;
		}
		// update pin state
		s = i; 
		// set another delay
		t.reset();
	}
	return 0;
}

int16_t microlib_Btn::pressed(void){
	if(q){
		q = 0; // the call clears the pressed status
		return 1;
	}
	return 0;
}

int16_t microlib_Btn::released(void){
	if(r){
		r = 0; // the call clears the released status
		return 1;
	}
	return 0;
}
