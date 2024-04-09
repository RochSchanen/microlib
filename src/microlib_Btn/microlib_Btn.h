// microlib_Btn.h

#ifndef microlib_Btn_h
#define microlib_Btn_h

#include "Arduino.h"
#include "microlib.h"

/*
	This defines the Button class of the microlib library.
 	
  	- the setup() method defines the digital input pin.
  	A refreshing rate of 0.100s is used by default to
   	prevent any possible spurious triggers due to
    	unexpected mechanical rebounce of the switch.
     
    	- the updt() method updates the buttons state values,
     	i.e. whether it has been pressed, released, or not. It
      	has to be called on a regular basis.
  	
   	- the pressed() method returns 1  (True) if the button
   	has been pressed (digital fall) since the previous call.
    	When called, the pressed() method clears the "pressed" flag.
     
     	- the released() method returns 1 (True) if the button
      	has been released (digital rise) since the previous call.
       	When called, the released() method clears the "released" flag.

	dependences: This library uses the micro library "microlib_Tmr".
 	The #include "microlib.h" at the top of this file should take
  	care of loading "microlib_Tmr" if necassary.
	
	todo:
	1) optimize the code by bypassing the standard "digitalRead()".
	2) combine button updates by reading a pin port only once:
		m = (1<<p);
		s = (PIND & m) >> p;
		i = (PIND & m) >> p;
*/

class microlib_Btn{

	private:

		// uint8_t m; 	// mask (not yet implemented)

		uint8_t p; 			// pin
		uint8_t s; 			// pin status
		uint8_t q; 			// button pressed status
		uint8_t r; 			// button released status
		uint8_t i; 			// readout value
		microlib_Tmr  t;		// timer

	public:

		int16_t setup(uint8_t pin);
		int16_t updt(void);
		int16_t pressed(void);
		int16_t released(void);
};

#endif
