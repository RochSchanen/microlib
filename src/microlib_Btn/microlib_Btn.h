// microlib_Btn.h

#ifndef microlib_Btn_h
#define microlib_Btn_h

#include "Arduino.h"
#include "microlib.h"

/*
	Button micro library: setup() defines the digital input
	pin. A refreshing rate at 0.100s maximum is used to
	prevent	switch rebounce triggers. updt() updates the
	buttons	status: wether it has been pressed or not.
	It has to be called on a regular basis. pressed()
	returns 1 if the button has been pressed (digital fall).
	pressed() clears the pressed flag. released() returns 1 
	if the button has been released (digital rise). pressed()
	clears the released flag.

	dependences: This library uses the micro library "microlib_Tmr".
	
	todo:
	1) optimize the code by bypassing digitalRead().
	2) combine button updates by reading a pin port only once:
		m = (1<<p);
		s = (PIND & m) >> p;
		i = (PIND & m) >> p;
*/

class microlib_Btn{

	private:

		// uint8_t m; 	// mask

		uint8_t p; 			// pin
		uint8_t s; 			// pin status
		uint8_t q; 			// button pressed status
		uint8_t r; 			// button released status
		uint8_t i; 			// readout value
		microlib_Tmr  t;	// timer

	public:

		int16_t setup(uint8_t pin);
		int16_t updt(void);
		int16_t pressed(void);
		int16_t released(void);
};

#endif
