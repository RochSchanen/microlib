// #define MICROLIB_TWI_DEBUG 1

/* this version of the the I2C does not use interrupts. The Interrupt
 * flag is tested manually but does not generate interruptions. The
 * TWIE bit is left OFF. This library does not handle SLAVE operations.
 * The microcontroller has to be the MASTER TRANSFER (MT) or MASTER
 * READ (MR).
 */

#include "Arduino.h"
#include "microlib.h"

// the following definitions and macros are found from "Arduino.h" 

// SFR stands for "Special Function Registers"
// "_BV"   "sfr_defs.h" macro : compute bit value

// "TWSR"  "iom328p.h"  macro : TW Status Register (with PreScaler)
// "TWDR"  "iom328p.h"  macro : TW Data Register
// "TWBR"  "iom328p.h"  macro : TW Bit Rate Register
// "TWCR"  "iom328p.h"  macro : TW Control Register
// "TWEN"  "iom328p.h"  def   : TWCR Enable Bit
// "TWINT" "iom328p.h"  def   : TWCR Interrupt Bit
// "TWIE"  "iom328p.h"  def   : TWCR Interrupt Enable Bit
// "TWSTA" "iom328p.h"  def   : TWCR Start bit
// "TWSTO" "iom328p.h"  def   : TWCR Stop bit
// "TWEA"  "iom328p.h"  def   : TWCR Enable Acknowledge bit

/* The following constant values were found in the following document: 
 * "Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf"
 * see p.260 in section "TWI - 2-wire Serial Interface.
 */

// meaning of some of the notation used:

//  _MT_ : Master Transmitter
//  _MR_ : Master Receiver
//  _SLA_: Slave Address
//  _DAT_: Data
//  _ACK : Acknowledged
//  _NCK : Not Acknowledged
//  TWSR : Two Wire State   Register
//  TWCR : Two Wire Control Register

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
#define TWSR_MASK       0xF8 // (mask prescaler)

#define TWCR_SEI    _BV(TWEN) |  _BV(TWINT)
#define TWCR_START   TWCR_SEI |  _BV(TWSTA)
#define TWCR_STOP    TWCR_SEI |  _BV(TWSTO)
#define TWCR_MT_SEND TWCR_SEI
#define TWCR_MR_ACK  TWCR_SEI |  _BV(TWEA)
#define TWCR_MR_NCK  TWCR_SEI & ~_BV(TWEA)

// buffer pointers stack (for multiple xfer calls)
// the stack is cyclic with a finite length.
// TW_MAX_CALLS is defined in microlib_TWi.h

uint8_t *TW_DATA_BUFFERS[TW_MAX_CALLS];
uint8_t *TW_CMDS_BUFFERS[TW_MAX_CALLS];
uint8_t  TW_ADDR_BUFFER[ TW_MAX_CALLS];

// pointer to the last entry
uint8_t  TW_BUFFER_LAST = 0; 

// local copies of the buffer pointers for code clarity
uint8_t *TW_DATA_BUFFER;
uint8_t *TW_CMDS_BUFFER;

// pointer to the current byte (DATA) and command (CMDS) in the buffer
uint8_t  TW_BUFFER_INDEX;

// temp vars computed once for any new device address  
uint8_t  TWDR_SLA_W;  // send address byte & write bit command
uint8_t  TWDR_SLA_R;  // send address byte & read  bit command
uint8_t  TW_DIR; // copy of one of the two previous variables

// TWI state (finite state machine)
uint8_t TW_STATUS;

// pointer to the current buffer being transfered in the stack
uint8_t TW_BUFFER_CURRENT = 0;

microlib_TWi::microlib_TWi(void){

  // TW_FREQ is defined in microlib_TWi.h

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

  #ifdef MICROLIB_TWI_DEBUG
  uint8_t db1, db2, db3, db4, db5, db6, db7;
  #endif

int16_t microlib_TWi::updt(void){

  /* Here the interrupt flag "TWINT" of the control register TWCR is
   * tested. If it is high, some action is requested by the bus
   * controller. The state of the bus is then given by the status
   * register TWSR.
   */

  #ifdef MICROLIB_TWI_DEBUG
  db1=0; db2=0; db3=0; db4=0; db5=0; db6=0; db7=0;
  #endif

  if (TWCR & _BV(TWINT)){

    TW_STATUS = TWSR & TWSR_MASK;

    switch(TW_STATUS){
      
      // started
      case TWSR_START:
      case TWSR_RESTART:
      
        // SEND ADDRESS + W/R
        TWDR = TW_DIR;
        TWCR = TWCR_MT_SEND;
        
        #ifdef MICROLIB_TWI_DEBUG
        db1+=1;
        #endif
        break;

      // Address sent with Write bit, and Acknowledged bit received
      case TWSR_MT_SLA_ACK:
      
        // SEND DATA BYTE
        TWDR = TW_DATA_BUFFER[TW_BUFFER_INDEX];
        TWCR = TWCR_MT_SEND;
        
        #ifdef MICROLIB_TWI_DEBUG
        db2+=1;
        #endif
        break;
      
      // Data byte sent, with Acknowledged bit received
      case TWSR_MT_DAT_ACK:
        
        // stop bit flagged ?
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          
          // SEND STOP
          TWCR = TWCR_STOP;
          // free buffer
          TW_CMDS_BUFFER[0] |= TW_READY;
          // clear buffer pointers
          TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
          // jump to next buffer on the cyclic stack
          TW_BUFFER_CURRENT++;
          TW_BUFFER_CURRENT %= TW_MAX_CALLS;

          // start next buffer transmission ?
          if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
            // temporary copy of the current buffer pointer
            TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
            TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
            // point to first data and command in the buffer
            TW_BUFFER_INDEX = 0;
            // compute write and read register flag values
            TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 0;
            TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 1;
            // select which of the write or read register to use
            if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
            if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
            // SEND START
            while(TWCR & _BV(TWSTO)){} // wait for stop bit settling
            TWCR = TWCR_START;
            }
                    
          else{ // all buffers are empty
            // FREE THE BUS
            while(TWCR & _BV(TWSTO)){} // wait for stop bit settling
            TW_STATUS = TW_FREE;}}

        else{ // continue with buffer 
          TW_BUFFER_INDEX++;

          // continue with writing
          if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_WRITE){
            // SEND DATA BYTE
            TWDR = TW_DATA_BUFFER[TW_BUFFER_INDEX];
            TWCR = TWCR_MT_SEND;}

          else{// switch to reading
            // SEND RE-START
            TW_DIR = TWDR_SLA_R;
            TWCR = TWCR_START;
            }
          }
          
        #ifdef MICROLIB_TWI_DEBUG
        db3+=1;
        #endif
        break;

      // Address sent with Read bit, and Acknowledged bit received     
      case TWSR_MR_SLA_ACK:
      
        // stop bit flagged ?
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_STOP){
          // READ BYTE + bit NACK
          TWCR = TWCR_MR_NCK;}
        
        // continue reading
        else if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_READ){
          // READ BYTE + bit ACK
          TWCR = TWCR_MR_ACK;}
          
        // stop reading (but continue transfer)  
        else{
          // READ BYTE + bit NACK
          TWCR = TWCR_MR_NCK;}
          
        #ifdef MICROLIB_TWI_DEBUG  
        db4+=1;
        #endif
        break;

      // Data byte received, with Acknowledged bit sent
      case TWSR_MR_DAT_ACK:
      
        // collect data into buffer
        TW_DATA_BUFFER[TW_BUFFER_INDEX] = TWDR;
        TW_BUFFER_INDEX++;
        
        // stop bit flagged ?
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          // READ BYTE + bit NACK
          TWCR = TWCR_MR_NCK;}
          
        // continue reading
        else if(TW_CMDS_BUFFER[TW_BUFFER_INDEX+1] & TW_READ){
          // READ BYTE + bit ACK
          TWCR = TWCR_MR_ACK;}
        
        // stop reading (but continue transfer)
        else{
          // READ BYTE + bit NACK
          TWCR = TWCR_MR_NCK;}
          
        #ifdef MICROLIB_TWI_DEBUG
        db5+=1;
        #endif
        break;

      // Data byte received, with Not Acknowledged bit sent
      case TWSR_MR_DAT_NCK:
        
        // collect last byte read
        TW_DATA_BUFFER[TW_BUFFER_INDEX] = TWDR;
        
        // stop bit flagged ?
        if(TW_CMDS_BUFFER[TW_BUFFER_INDEX] & TW_STOP){
          
          // SEND STOP
          TWCR = TWCR_STOP;
          // free buffer
          TW_CMDS_BUFFER[0] |= TW_READY;
          // clear buffer pointers
          TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
          TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
          // jump to next buffer on the cyclic stack  
          TW_BUFFER_CURRENT++;
          TW_BUFFER_CURRENT %= TW_MAX_CALLS;
          
          // start next buffer transmission ?
          if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
            // temporary copy of the current buffer pointer
            TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
            TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
            // point to first data and command in the buffer
            TW_BUFFER_INDEX = 0;
            // compute write and read register flag values
            TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 0;
            TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT] << 1) | 1;
            // select which of the write or read register to use
            if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
            if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
            // SEND START
            while(TWCR & _BV(TWSTO)){}// wait for stop bit settling
            TWCR = TWCR_START;}
          
          else{ // all buffers are empty
            // FREE THE BUS
            while(TWCR & _BV(TWSTO)){} // wait for stop bit settling
            TW_STATUS = TW_FREE;}}
        
        else{ // continue transfer
          TW_BUFFER_INDEX++;
          
          // switch to writing
          TW_DIR = TWDR_SLA_W;
          // SEND RE-START
          TWCR = TWCR_START;}
          
        #ifdef MICROLIB_TWI_DEBUG  
        db6+=1;
        #endif
        break;

      // unexpected states... device sent Not Acknowdledge signal
      case TWSR_MR_SLA_NCK: // Addressed Sent+R, Not Acknowledge 
      case TWSR_MT_SLA_NCK: // Addressed Sent+W, Not Acknowledge
      case TWSR_MT_DAT_NCK: // Data Sent, Not Acknowledge
        // SEND STOP
        TWCR = TWCR_STOP;
        // free buffer and raise error flag
        TW_CMDS_BUFFER[0] |= (TW_READY | TW_ERROR);
        // clear buffer pointers
        TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = 0;
        TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = 0;
        // jump to next buffer on the cyclic stack
        TW_BUFFER_CURRENT++;
        TW_BUFFER_CURRENT %= TW_MAX_CALLS;
        
        // start next buffer transmission ?
        if(TW_CMDS_BUFFERS[TW_BUFFER_CURRENT]){
          // temporary copy of the current buffer pointer
          TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
          TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
          // point to first data and command in the buffer
          TW_BUFFER_INDEX = 0;
          // compute write and read register flag values
          TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 0;
          TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 1;
          // select which of the write or read register to use
          if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
          if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
          // SEND START
          while(TWCR & _BV(TWSTO)){} // wait for stop bit settling
          TWCR = TWCR_START;}
        
        else{// all buffers are empty
          // FREE THE BUS
          while(TWCR & _BV(TWSTO)){} // wait for stop bit settling
          TW_STATUS = TW_FREE;}
          
        #ifdef MICROLIB_TWI_DEBUG
        db7+=1;
        #endif
      }
      
    #ifdef MICROLIB_TWI_DEBUG  
    if (db1) Serial.print("1 ");
    if (db2) Serial.print("2 ");
    if (db3) Serial.print("3 ");
    if (db4) Serial.print("4 ");
    if (db5) Serial.print("5 ");
    if (db6) Serial.print("6 ");
    if (db7) Serial.print("7 ");
    if (TW_STATUS == TW_FREE) Serial.println();
    #endif
  }

  return 0;
}

uint8_t microlib_TWi::xfer(uint8_t address, uint8_t *data, uint8_t *cmds){

  // mask all but write and read flags
  cmds[0] &= (TW_WRITE | TW_READ);

  if (TW_STATUS){ // the bus is busy: ADD TASK TO THE BUFFER STACK

    // increment pointer to stack last added buffer
    TW_BUFFER_LAST++;
    TW_BUFFER_LAST %= TW_MAX_CALLS; // cycle back to start?
    // overfilling: push the current transfer to complete
    if (TW_BUFFER_LAST == TW_BUFFER_CURRENT){
      // raise overfull flag?
      cmds[0] |= TW_OVERFULL;
      // update the bus until there is some free space
      while(TW_BUFFER_LAST == TW_BUFFER_CURRENT) updt();
      // clear overfull flag?
      cmds[0] &= ~TW_OVERFULL;
      }
    // add address and buffer pointers to stack
    TW_ADDR_BUFFER[TW_BUFFER_LAST] = address;
    TW_CMDS_BUFFERS[TW_BUFFER_LAST] = cmds;
    TW_DATA_BUFFERS[TW_BUFFER_LAST] = data;
    }
  
  else{ // The bus is free: ADD TASK AND START XFER

    // add address and buffer pointers to stack
    TW_ADDR_BUFFER[TW_BUFFER_CURRENT] = address;
    TW_CMDS_BUFFERS[TW_BUFFER_CURRENT] = cmds;
    TW_DATA_BUFFERS[TW_BUFFER_CURRENT] = data;
    // temporary copy of the current buffer pointer
    TW_CMDS_BUFFER  = TW_CMDS_BUFFERS[TW_BUFFER_CURRENT];
    TW_DATA_BUFFER  = TW_DATA_BUFFERS[TW_BUFFER_CURRENT];
    // point to first data and command in the buffer
    TW_BUFFER_INDEX = 0;
    // compute write and read register flag values
    TWDR_SLA_W = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 0;
    TWDR_SLA_R = (TW_ADDR_BUFFER[TW_BUFFER_CURRENT]<<1) | 1;
    // select which of the write or read register to use
    if(TW_CMDS_BUFFER[0] == TW_WRITE) TW_DIR = TWDR_SLA_W;
    if(TW_CMDS_BUFFER[0] == TW_READ)  TW_DIR = TWDR_SLA_R;
    // the bus was free, the current buffer is also the last
    TW_BUFFER_LAST = TW_BUFFER_CURRENT;
    // update bus state
    TW_STATUS = TWSR_BUSY;
    // SEND START
    TWCR = TWCR_START;
    }

  return 0;
}

uint8_t microlib_TWi::wait(uint8_t *cmds){
  while((cmds[0] & TW_READY) == 0) updt();
  return 0;
}

uint8_t microlib_TWi::ready(uint8_t *cmds){
  return (cmds[0] & TW_READY) == TW_READY;
}
