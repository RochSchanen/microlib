#include "Arduino.h"
#include "microlib.h"

int16_t microlib_Btn::setup(uint8_t pin){
	// setup pin
	p = pin; pinMode(p, INPUT_PULLUP);
	// setup initial state values
	s = digitalRead(p); q = 0; r = 0;
	// setup anti-rebounce timer
	t.setup(100000); 	// use 100ms by default
	t.reset(); 		// reset timer
	return 0;
}

int16_t microlib_Btn::updt(void){
	// wait for timer
	if (t.ready()){
		i = digitalRead(p);
		// has the pin changed its state?
		if (i == s) return 0; // no -> done
		// yes -> update flags
		switch(i){
			case 0: // the button has been pressed
				q = 1; // set the pressed flag
				break;
			case 1: // the button has been released
				r = 1; // set the released flag
				break;
		}
		// update pin state
		s = i; 
		// reset timer
		t.reset();
	}
	// done
	return 0;
}

int16_t microlib_Btn::pressed(void){
	if(q){
		q = 0; // clear the pressed flag when called
		return 1; 
	}
	return 0;
}

int16_t microlib_Btn::released(void){
	if(r){
		r = 0; // clear the released flag when called 
		return 1;
	}
	return 0;
}
