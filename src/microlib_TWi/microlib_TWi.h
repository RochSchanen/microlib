// microlibmicrolib_TWi.h

#ifndef microlib_TWi_h
#define microlib_TWi_h

#include "Arduino.h"

/* microlib_TWi library Header file:
 * This mini library is used to transfer data on a "Two Wire Interface" 
 * bus using the ATmega328P. It is non-blocking: A call to "xfer()"
 * starts the data transfer and returns immediately. Fast and regular
 * calls to the "updt()" method are necessary to complete the transfer
 * byte per byte until completion. The "wait(cmds)" method is blocking
 * and call "updt()" until completion of the "cmds" buffer transfer.
 * when a xfer is initiated while the previous transfer is still
 * ongoing, it is put on a stack of transfers to be processed. 
 * If the maximum number of buffers in the stack is reached. The system
 * will eventually waits until some space is freed. However, this will
 * slow down other part of the code and is not advisable. It is
 * possible to increased the maximum number of buffers, or to try
 * reducing the data flow across the I2C channel.
 * 
 * A documentat on the ATmega328P is found in the folder "extras": 
 * Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
 */

// You can adjust the following parameters:
//#define TW_FREQ 400000L // 100KHz is the default value
#define TW_FREQ 100000L // 100KHz is the default value
#define TW_MAX_CALLS 32 // Maximum competing buffers

// Control codes of this TWI implementation

#define TW_WRITE    (1<<0)
#define TW_READ     (1<<1)
#define TW_STOP     (1<<2)
#define TW_READY    (1<<3)
#define TW_ERROR    (1<<4)
#define TW_OVERFULL (1<<5)

// mini class for controlling the bus:
class microlib_TWi{

  public:

    microlib_TWi(void);

    int16_t updt(void);
    uint8_t xfer(uint8_t address, uint8_t *data, uint8_t *cmds);
    uint8_t wait(uint8_t *cmds);    // wait for the end of the transfer
    uint8_t ready(uint8_t *cmds);   // true if the transfer is done

};

#endif
