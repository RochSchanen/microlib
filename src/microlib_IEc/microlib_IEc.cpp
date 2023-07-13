#include "Arduino.h"
#include "microlib.h"

// tables for fast analysis and update of the signals A and B
// the value zero points to the state 0, i.e. when a level transition
// has been missed. The state 0 table is used to reset the quadrant.

const uint8_t microlib_IEc::Q[5][4]={ 
  { 1, 2, 4, 3},  // Missed transition           (state #0)
  { 1, 2, 4, 0},  // LOW  B, LOW  A <=> 00 <=> 0 (state #1)
  { 1, 2, 0, 3},  // LOW  B, HIGH A <=> 01 <=> 1 (state #2)
  { 0, 2, 4, 3},  // HIGH B, HIGH A <=> 11 <=> 3 (state #3)
  { 1, 0, 4, 3}}; // HIGH B, LOW  A <=> 10 <=> 2 (state #4)

// one cycle runs from state 1 to state 4. Cycles are counted when
// passing through cycle boundaries: below state 1 or above state 4.

const int8_t microlib_IEc::C[5][4]={ 
  { 0, 0, 0, 0},  // 0: 
  { 0, 0,-1, 0},  // 1: Decr by One Cycle 
  { 0, 0, 0, 0},  // 2: 
  { 0, 0, 0, 0},  // 3: 
  {+1, 0, 0, 0}}; // 4: Incr by One Cycle

int16_t microlib_IEc::setup(uint8_t A, uint8_t B, uint8_t D){
  
  a = 1 << A; // compute bit weight of signal A
  b = 1 << B; // compute bit weight of signal B
  
  // init cycle count
  c = 0; 

  // init quadrant value
  i1 = (D & a) > 0;             // read state of A
  i2 = (D & b) > 0;             // read state of B
  i  = (i1 << 0) | (i2 << 1);   // compute quadrant (1)
  q = Q[0][i];                  // compute quadrant (2)
  
  return 0;
}

int16_t microlib_IEc::updt(uint8_t D){
  
  // compute new quadrant value and adjust cycle count
  i1 = (D & a) > 0;             // read state of A
  i2 = (D & b) > 0;             // read state of B
  i  = (i1 << 0) | (i2 << 1);   // compute quadrant (1)
  c += C[q][i];                 // incr/decr cycle count
  q  = Q[q][i];                 // compute quadrant (2)
  
  // missed pulse -> reset quadrant
  if (q==0) {q = Q[0][i];}
  
  // coerce cycle to boundaries
  if (c<c1) c = c1;
  if (c>c2) c = c2;

  return 0;
}

int16_t microlib_IEc::get(void){
  return c;
}

int16_t microlib_IEc::set(int16_t value){

  c = value;

  // coerce cycle to boundaries
  if (c<c1) c = c1;
  if (c>c2) c = c2;

  return 0;
}

int16_t microlib_IEc::set(int16_t value, int16_t min, int16_t max){

  c = value;
  c1 = min;
  c2 = max;

  // coerce cycle to boundaries
  if (c<c1) c = c1;
  if (c>c2) c = c2;

  return 0;
}
