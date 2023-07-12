// microlib_Tmr.h

#ifndef microlib_Tmr_h
#define microlib_Tmr_h

#include "Arduino.h"

/*
	Timer micro library: setup() sets the timer delay. reset() resets
	the timer. ready() returns true if the delay has been reached.
	
	dependences: This library has no dependencies.
	
*/

class microlib_Tmr{

	private:

		uint32_t d, l;

	public:

		int16_t setup(uint32_t delay);
		int16_t reset(void);
		int16_t ready(void);
};

#endif
