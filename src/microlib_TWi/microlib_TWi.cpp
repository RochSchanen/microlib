/* this version of the the I2C does not use interrupts. The Interrupt
 * flag is tested manually but does not generate interruptions. The
 * TWIE bit is left OFF. This library does not handle SLAVE operations.
 * The microcontroller has to be the MASTER TRANSFER (MT) or MASTER
 * READ (MR).
 */

#include "Arduino.h"
#include "microlib.h"

// the following definitions and macros are found from "Arduino.h" 

// "_BV"   "sfr_defs.h" macro : compute bit value
// "TWSR"  "iom328p.h"  macro : TW Status Register (Includes PreScaler)
// "TWDR"  "iom328p.h"  macro : TW Data Register
// "TWBR"  "iom328p.h"  macro : TW Bit Rate Register
// "TWCR"  "iom328p.h"  macro : TW Control Register
// "TWEN"  "iom328p.h"  def   : TWCR Enable Bit
// "TWINT" "iom328p.h"  def   : TWCR Interrupt Bit
// "TWIE"  "iom328p.h"  def   : TWCR Interrupt Enable Bit
// "TWSTA" "iom328p.h"  def   : TWCR Start bit
// "TWSTO" "iom328p.h"  def   : TWCR Stop bit
// "TWEA"  "iom328p.h"  def   : TWCR Enable Acknowledge bit

/* The following constant were found in: 
 * "Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf"
 * see p.260 in section "TWI - 2-wire Serial Interface.
 */

#define TW_FREE         0x00
#define TWSR_BUSY       0x01
#define TWSR_START      0x08
#define TWSR_RESTART    0x10
#define TWSR_MT_SLA_ACK 0x18
#define TWSR_MT_SLA_NCK 0x20
#define TWSR_MT_DAT_ACK 0x28
#define TWSR_MT_DAT_NCK 0x30
#define TWSR_MR_SLA_ACK 0x40
#define TWSR_MR_SLA_NCK 0x48
#define TWSR_MR_DAT_ACK 0x50
#define TWSR_MR_DAT_NCK 0x58
#define TWSR_MASK       0xF8

#define TWCR_SEI    _BV(TWEN) |  _BV(TWINT)
#define TWCR_START   TWCR_SEI |  _BV(TWSTA)
#define TWCR_STOP    TWCR_SEI |  _BV(TWSTO)
#define TWCR_MT_SEND TWCR_SEI
#define TWCR_MR_ACK  TWCR_SEI |  _BV(TWEA)
#define TWCR_MR_NCK  TWCR_SEI & ~_BV(TWEA)

// Control codes of this TWI implementation

#define TW_WRITE    (1<<0)
#define TW_READ     (1<<1)
#define TW_STOP     (1<<2)
#define TW_READY    (1<<3)
#define TW_ERROR    (1<<4)
#define TW_OVERFULL (1<<5)

// buffer variables

uint8_t *TW_DATA_BUFFERS[TW_MAX_CALLS];
uint8_t *TW_CMDS_BUFFERS[TW_MAX_CALLS];
uint8_t  TW_ADDR_BUFFER[TW_MAX_CALLS];
uint8_t  TW_BUFFER_LAST = 0;
uint8_t *TW_DATA_BUFFER;
uint8_t *TW_CMDS_BUFFER;
uint8_t  TW_BUFFER_INDEX;

// direction status variables

uint8_t  TWDR_SLA_W;
uint8_t  TWDR_SLA_R;
uint8_t  TW_DIR;

// TWI state variables

uint8_t TW_STATUS;
uint8_t TW_BUFFER_CURRENT = 0;

microlib_TWi::microlib_TWi(void){

  // SET BUS SPEED
  TWBR = ((F_CPU / TW_FREQ) - 16) / 2;

  // SET INTERNAL PULL UP RESISTORS
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
  
  // CLEAR BUFFERS MEMORY
  for(uint8_t i=0; i<TW_MAX_CALLS; i++){
    TW_ADDR_BUFFER[i]  = 0;
    TW_CMDS_BUFFERS[i] = 0;
    TW_DATA_BUFFERS[i] = 0;}

  // INIT PARAMETERS
  TW_STATUS = TW_FREE;

}

// uint8_t S1, S2, S3, S4, S5, S6; // debug

int16_t microlib_TWi::updt(void){

  /* Here the interrupt flag "TWINT" is tested. If it is on, some
   * action is requested by the bus controller. The state of the bus
   * is then given by the status register TWSR.
   */

  if (TWCR & _BV(TWINT)){

    TW_STATUS = TWSR & TWSR_MASK;

    switch(TW_STATUS){
      
      // START/RESTART ACKNOWLEDGED

      case TWSR_START:
      case TWSR_RESTART:                                      // S1++;

        TWDR = TW_DIR;
        TWCR = TWCR_MT_SEND;
        break;
      
      // ADDRESS + WRITE BIT AKNOWLEDGED

      case TWSR_MT_SLA_ACK:                                   // S2++;
        TWDR = TW_DATA_BUFFER[TW_BUFFER_INDEX];
        TWCR = TWCR_MT_SEND;
        break;
      
      // ONE BYTE SENT AKNOLEDGED

      case TWSR_MT_DAT_ACK:                                   // S3++;
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          TWCR = TWCR_STOP;
          TW_CMDS_BUFFER[0] |= TW_READY;
          TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_BUFFER_CURRENT++;
          TW_BUFFER_CURRENT %= TW_MAX_CALLS;
          if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
            TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
            TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
            TW_BUFFER_INDEX = 0;
            TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 0;
            TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 1;
            if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
            if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
            while(TWCR & _BV(TWSTO)){}
            // S1 = S2 = S3 = S4 = S5 = S6 = 0;
            TWCR = TWCR_START;}
          else{
            while(TWCR & _BV(TWSTO)){}
            TW_STATUS = TW_FREE;}}
        else{
          TW_BUFFER_INDEX++;
          if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_WRITE){
            TWDR = TW_DATA_BUFFER[TW_BUFFER_INDEX];
            TWCR = TWCR_MT_SEND;}
          else{
            TW_DIR = TWDR_SLA_R;
            TWCR = TWCR_START;}}
        break;

      // ADDRESS + READ BIT AKNOWLEDGED
      
      case TWSR_MR_SLA_ACK:                                   // S4++;
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_STOP){
          TWCR = TWCR_MR_NCK;}
        else if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_READ){
          TWCR = TWCR_MR_ACK;}
        else{
          TWCR = TWCR_MR_NCK;}
        break;

      // ONE BYTE RECEIVED ACKNOLEDGED

      case TWSR_MR_DAT_ACK:                                   // S5++;
        TW_DATA_BUFFER[TW_BUFFER_INDEX] = TWDR;
        TW_BUFFER_INDEX++;
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          TWCR = TWCR_MR_NCK;}
        else if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_READ){
          TWCR = TWCR_MR_ACK;}
        else{
          TWCR = TWCR_MR_NCK;}
        break;

      // LAST BYTE RECEIVED 

      case TWSR_MR_DAT_NCK:                                   // S6++;
        TW_DATA_BUFFER[TW_BUFFER_INDEX] = TWDR;
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          TWCR = TWCR_STOP;
          TW_CMDS_BUFFER[0] |= TW_READY;
          TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_BUFFER_CURRENT++;
          TW_BUFFER_CURRENT %= TW_MAX_CALLS;
          if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
            TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
            TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
            TW_BUFFER_INDEX = 0;
            TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 0;
            TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 1;
            if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
            if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
            while(TWCR & _BV(TWSTO)){}
            // S1 = S2 = S3 = S4 = S5 = S6 = 0;
            TWCR = TWCR_START;}
          else{
            while(TWCR & _BV(TWSTO)){}
            TW_STATUS = TW_FREE;}}
        else{
          TW_BUFFER_INDEX++;
          TW_DIR = TWDR_SLA_W;
          TWCR = TWCR_START;}
        break;

      // ADDRESS or DATA NOT ACKOWLEDGED (ERROR)

      case TWSR_MR_SLA_NCK:
      case TWSR_MT_SLA_NCK:
      case TWSR_MT_DAT_NCK: 
        TWCR = TWCR_STOP;
        TW_CMDS_BUFFER[0] |= (TW_READY | TW_ERROR);
        TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
        TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
        TW_BUFFER_CURRENT++;
        TW_BUFFER_CURRENT %= TW_MAX_CALLS;
        if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
          TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
          TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
          TW_BUFFER_INDEX = 0;
          TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 0;
          TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 1;
          if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
          if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
          while(TWCR & _BV(TWSTO)){}
          // S1 = S2 = S3 = S4 = S5 = S6 = 0;
          TWCR = TWCR_START;}
        else{
          while(TWCR & _BV(TWSTO)){}
          TW_STATUS = TW_FREE;}
      }
  }

  return 0;
}

uint8_t microlib_TWi::xfer(uint8_t address, uint8_t *data, uint8_t *cmds){

  // MASK "RETURN FLAGS"
  cmds[0] &= (TW_WRITE | TW_READ);

  // CHECK BUS STATUS
  if (TW_STATUS){

    // BUSY BUS: ADD TASK TO THE BUFFER LIST

    TW_BUFFER_LAST++;
    TW_BUFFER_LAST %= TW_MAX_CALLS;
    if (TW_BUFFER_LAST == TW_BUFFER_CURRENT){
      cmds[0] |= TW_OVERFULL;
      // update the bus until there is some free space
      while(TW_BUFFER_LAST == TW_BUFFER_CURRENT) updt();}
    TW_ADDR_BUFFER[TW_BUFFER_LAST] = address;
    TW_CMDS_BUFFERS[TW_BUFFER_LAST] = cmds;
    TW_DATA_BUFFERS[TW_BUFFER_LAST] = data;}
  
  else{
  
    // FREE BUS: ADD TASK TO THE CURRENT BUFFER AND START XFER

    TW_ADDR_BUFFER[TW_BUFFER_CURRENT] = address;
    TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = cmds;
    TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = data;
    TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
    TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
    TW_BUFFER_INDEX = 0;
    TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 0;
    TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 1;
    if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
    if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
    TW_BUFFER_LAST = TW_BUFFER_CURRENT;
    // S1 = S2 = S3 = S4 = S5 = S6 = 0;
    TW_STATUS = TWSR_BUSY;
    TWCR = TWCR_START;}

  return 0;
}

uint8_t microlib_TWi::wait(uint8_t *cmds){
  while((cmds[0] & TW_READY) == 0) updt();
  return 0;
}

uint8_t microlib_TWi::ready(uint8_t *cmds){
  return (cmds[0] & TW_READY) == TW_READY;
}

// debug:
// Serial.print("\nN:");  Serial.print(TW_BUFFER_CURRENT);
// Serial.print(" ");     Serial.print(S1);
// Serial.print(" ");     Serial.print(S3); 
// Serial.print(" ");     Serial.print(S4); 
// Serial.print(" ");     Serial.print(S5); 
// Serial.print(" ");     Serial.print(S6); 
// Serial.print(" S:0x"); Serial.print(TW_STATUS, HEX);
// Serial.print(" C:0b"); Serial.print(cmds[0], BIN);
