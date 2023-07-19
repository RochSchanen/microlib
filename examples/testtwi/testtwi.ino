#include "microlib.h"

///////////////////////////////////////////////////////////////////////
// compile , upload and monitor:
// arduino-cli compile --fqbn arduino:avr:mega testmicrolib/
// arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:avr:mega testmicrolib/
// arduino-cli monitor -p /dev/ttyACM0
///////////////////////////////////////////////////////////////////////

microlib_TWi bus_handle = microlib_TWi();

// microlib_TWi is tested on a tiny Elegoo RTC module based on the
// DS1397 device. See the "DS1307 Serial Real Time Clock Module.pdf"
// file in "extras/" for a detailed description of the device. Two
// general methods of buffer transfer are used to change the RTC
// registers (one to write, one to read). See below.

# define MAX_TRANSFER_LENGTH 32

// Read one or multiple byte to a buffer from a given device at a given address.
// The call is blocking and returns only when the transfer is completed
uint8_t readAddressedBuffer(uint8_t ba, uint8_t ra, void* p, uint8_t n){

  // ba: bus address
  // ra: register address
  // p: pointer to the data buffer
  // n: data buffer size in bytes

  // SETUP BUS COMMANDS
  uint8_t i, c[MAX_TRANSFER_LENGTH];  // 1 byte address + n data bytes
  c[0] = TW_WRITE;                    // send address (send 1 byte)
  for(i=1;i<n;i++) c[i] = TW_READ ;   // read data (read n-1 bytes)
  c[n] = TW_READ | TW_STOP;           // read last byte + STOP

  // SETUP DATA
  uint8_t d[MAX_TRANSFER_LENGTH];     // 1 byte address + n data bytes
  d[0] = ra;                          // send address (send 1 byte)
  for(i=1;i<n+1;i++) d[i] = 0x00 ;    // read data (read n bytes)

  // TRANSFER
  bus_handle.xfer(ba, d, c);          // start transfer
  bus_handle.wait(c);                 // wait for completion

  // COPY DATA TO BUFFER
  uint8_t *b = p;
  for(i=0;i<n;i++) b[i] = d[i+1] ;    // copy n bytes

  // DONE
  return 0;
}

// Write one or multiple byte from a buffer to a given device at a given address.
// The call is blocking and returns only when the transfer is completed
uint8_t writeAddressedBuffer(uint8_t ba, uint8_t ra, void* p, uint8_t n){

  // ba: bus address
  // ra: register address
  // p: pointer to the data buffer
  // n: data buffer size in bytes

  // SETUP BUS COMMANDS
  uint8_t i, c[MAX_TRANSFER_LENGTH];  // 1 byte address + n data bytes
  c[0] = TW_WRITE;                    // send address (write 1 byte)
  for(i=1;i<n;i++) c[i] = TW_WRITE;   // send data (write n-1 bytes)
  c[n] = TW_WRITE | TW_STOP;          // write last byte + STOP

  // SETUP DATA
  uint8_t *b = p;                     // byte pointer to data buffer
  uint8_t d[MAX_TRANSFER_LENGTH];     // 1 byte address + n data bytes
  d[0] = ra;                          // send address (write 1 byte)
  for(i=0;i<n;i++) d[i+1] = b[i];     // write data (n bytes)

  // TRANSFER
  bus_handle.xfer(ba, d, c);          // start transfer
  bus_handle.wait(c);                 // wait for completion

  // DONE
  return 0;
}

///////////////////////////////////////////////////////////////////////

// DEFINITIONS SPECIFIC TO THE DS1397

#define DS1307_TWI_ADDRESS (0x68)

// registers addresses
#define DS1307_REGISTERS_START_ADDRESS  (0x00)
#define DS1307_RAM_START_ADDRESS        (0x08)

// bits definitions for registers:

#define DS1307_REG_0_BIT_CH   1<<7   // CH: clock is halted when high

#define DS1307_REG_2_BIT_1224 1<<6   // mode 12 hours mode selected when high
#define DS1307_REG_2_BIT_AMPM 1<<5   // AM/PM flag: PM when high and 12 hours mode selected

#define DS1307_REG_7_BIT_OUT  1<<7   // OUT: output level when SQWE disabled
#define DS1307_REG_7_BIT_SQWE 1<<4   // SQWE: square wave enabled when high 
#define DS1307_REG_7_BIT_RS1  1<<1   // RS1: rate selector bit 1 (output frequency)
#define DS1307_REG_7_BIT_RS0  1<<0   // RS0: rate selector bit 0 (output frequency)

// rate selector values
// RS = 0b00:  0.001 [KHz]
// RS = 0b01:  4.096 [KHz]
// RS = 0b10:  8.192 [KHz]
// RS = 0b11: 32.768 [KHZ]

// registers structure
typedef struct {
    uint8_t REG_0;  // second + clock halt (CH)
    uint8_t REG_1;  // minutes
    uint8_t REG_2;  // hours + mode (12/24)
    uint8_t REG_3;  // day (day of the week)
    uint8_t REG_4;  // date (day of the month)
    uint8_t REG_5;  // month
    uint8_t REG_6;  // year
    uint8_t REG_7;  // control (OUT, SQWE, RS0, RS1)
  } DS1307_REGISTERS_STRUCT_TYPE;

///////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
  while (!Serial) ;

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("---init---");

  // DECLARE A REGISTER STRUCTURE BUFFER
  DS1307_REGISTERS_STRUCT_TYPE b;
  
  // READ REGISTER STRUCTURE:
  readAddressedBuffer(DS1307_TWI_ADDRESS, DS1307_REGISTERS_START_ADDRESS, &b, sizeof(b));
  
  // MODIFY INDIVIDUAL REGISTERS:
  // b.REG_0 |= DS1307_REG_0_BIT_CH; // Clock Hold
  b.REG_0 &= ~DS1307_REG_0_BIT_CH; // Clock Enabled
  
  // WRITE BACK REGISTER STRUCTURE:
  writeAddressedBuffer(DS1307_TWI_ADDRESS, DS1307_REGISTERS_START_ADDRESS, &b, sizeof(b));

}

void loop() {
    
  // DECLARE A REGISTER STRUCTURE BUFFER
  DS1307_REGISTERS_STRUCT_TYPE b;

  // READ REGISTER STRUCTURE:
  readAddressedBuffer(DS1307_TWI_ADDRESS, DS1307_REGISTERS_START_ADDRESS, &b, sizeof(b));

  // DISPLAY REGISTERS
  Serial.print(b.REG_0); Serial.print(" ");
  Serial.print(b.REG_1); Serial.print(" ");
  Serial.print(b.REG_2); Serial.print(" ");
  Serial.print(b.REG_3); Serial.print(" ");
  Serial.print(b.REG_4); Serial.print(" ");
  Serial.print(b.REG_5); Serial.print(" ");
  Serial.print(b.REG_6); Serial.print(" ");
  Serial.print(b.REG_7); Serial.print(" ");
  Serial.println();
  Serial.println("---");
  delay(1000);

}

/* Connect the device +5V, GND, SDA, and SCL pins. Forget the SQW pin.
 * Compile. 
 * Upload.
 * Monitor
 */
