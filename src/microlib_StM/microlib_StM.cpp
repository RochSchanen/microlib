#include "Arduino.h"
#include "microlib.h"

#define MTR_CONT_FWD (1<<0)       // continuous forward
#define MTR_CONT_REV (1<<1)       // continuous reversed
#define MTR_POWERED  (1<<2)       // powered

#define SbSt_BITS 4               // sub steps bit length
#define SbSt_LENG (1<<SbSt_BITS)  // sub steps number (=16)
#define SbSt_MASK (SbSt_LENG-1)   // sub steps bit mask (=15)

#define pattern_A1 3              // quadrant coil A1 pattern (=0011)
#define pattern_B1 9              // quadrant coil B1 pattern (=1001)
#define pattern_A2 (~pattern_A1)  // quadrant coil A2 pattern (=1100)
#define pattern_B2 (~pattern_B1)  // quadrant coil B2 pattern (=0110)

// get motor position (from step-tracking/step-integration)
int32_t microlib_StM::getpos(void){
  return p;}

// set new target
void microlib_StM::target(int32_t position){
  T = position;
  S = MTR_POWERED;
  return;}

// shift target value
void microlib_StM::shift(int32_t delta){
  T += delta;
  S = MTR_POWERED;
  return;}

// set state: forward direction
void microlib_StM::forward(void){
  T = p;
  S = MTR_CONT_FWD | MTR_POWERED;
  return;}

// set state: reverse direction
void microlib_StM::reverse(void){
  T = p;
  S = MTR_CONT_REV | MTR_POWERED;
  return;}

// set state: power on (torque on)
void microlib_StM::hold(void){
  T = p;
  S = MTR_POWERED;
  return;}

// set state: release power (torque off)
void microlib_StM::release(void){
  S = 0;
  return;}

// setup motor time and step intervals
void microlib_StM::interval(uint32_t microseconds, uint8_t substepsize){
  // time instervals between substep calls
  t.setup(microseconds);
  // size of the sub steps (in 1/16 of a full step?)
  d = substepsize;
  // "round" p to the "nearest" multiple of d:
  p /= d;
  p *= d;
  // done
  return;}

int16_t microlib_StM::setup(uint8_t PWA, uint8_t INA1, uint8_t INA2,
                            uint8_t PWB, uint8_t INB1, uint8_t INB2){
  // pin definitions
  pwa = PWA; ina1 = INA1; ina2 = INA2;
  pwb = PWB; inb1 = INB1; inb2 = INB2;
  // setup output mode and polarity startup values
  pinMode(INA1, OUTPUT); digitalWrite(INA1, LOW);
  pinMode(INA2, OUTPUT); digitalWrite(INA2, LOW);
  pinMode(INB1, OUTPUT); digitalWrite(INB1, LOW);
  pinMode(INB2, OUTPUT); digitalWrite(INB2, LOW);
  // setup output mode and power startup values
  pinMode(pwa, OUTPUT);  analogWrite(pwa, 0);
  pinMode(pwb, OUTPUT);  analogWrite(pwb, 0);
  // vars start up values
  q = 0;
  p = 0;
  S = 0;
  P = 0;
  T = p;
  // default motor intervals setup
  interval(1000L, 1);
  // reset timers
  t.reset();
  // done
  return 0;}

// power table from substep position within a quadrant (16 + 1 values)
static const uint8_t microlib_StM::PWM_MSB[]={
  0x00, 0x19, 0x32, 0x4a, 0x62, 0x78, 0x8e, 0xa2,
  0xb4, 0xc5, 0xd4, 0xe1, 0xec, 0xf4, 0xfa, 0xfe, 0xff};

int16_t microlib_StM::updt(void){

  // update motor state according to the state variables S, P, p, t

  // motor state is expected to be powered on
  if(S & MTR_POWERED){
    // the output power is already on
    if(P == 1){
      // time-interval timer is ringing
      if(t.ready()){
        // check if target reached
        if(p == T){
          // set new target to next substep interval
          if(S & MTR_CONT_FWD) T = p + d;
          if(S & MTR_CONT_REV) T = p - d;}
        // compute next motor position
        if(p < T) p += d; // if motor below target, increment
        if(p > T) p -= d; // if motor above target, decrement
        // compute quadrant
        q = (p >> SbSt_BITS) & 3;
        // compute substep position within quadrant
        switch(q){
          case 0: case 2: sa = p & SbSt_MASK; sb = SbSt_LENG - sa; break;
          case 1: case 3: sb = p & SbSt_MASK; sa = SbSt_LENG - sb; break;
          default: break;}
        // compute polarities from quadrant position and pattern definitions
        a1 = (pattern_A1 >> q) & 1; a2 = (pattern_A2 >> q) & 1;
        b1 = (pattern_B1 >> q) & 1; b2 = (pattern_B2 >> q) & 1;
        // compute substep power from substep position within the quadrant
        pa = PWM_MSB[sa];
        pb = PWM_MSB[sb];
        // update polarities and powers
        digitalWrite(ina1, a1); digitalWrite(ina2, a2); analogWrite(pwa, pa);
        digitalWrite(inb1, b1); digitalWrite(inb2, b2); analogWrite(pwb, pb);
        // reset time interval timer
        t.reset();}

    // motor state is expected to be powered on
    // but the power output is still off -> switch on
    }else{
      // compute quadrant
      q = (p >> SbSt_BITS) & 3;
      // compute substep position within quadrant
      switch(q){
        case 0: case 2: sa=p&SbSt_MASK; sb =  SbSt_LENG-sa; break;
        case 1: case 3: sb=p&SbSt_MASK; sa =  SbSt_LENG-sb; break;
        default: break;}
      // compute polarities from quadrant position and pattern definitions
      a1 = (pattern_A1 >> q) & 1; a2 = (pattern_A2 >> q) & 1;
      b1 = (pattern_B1 >> q) & 1; b2 = (pattern_B2 >> q) & 1;
      // compute substep power from substep position within the quadrant
      pa = PWM_MSB[sa];
      pb = PWM_MSB[sb];
      // set initial polarities and powers
      digitalWrite(ina1, a1); digitalWrite(ina2, a2); analogWrite(pwa, pa);
      digitalWrite(inb1, b1); digitalWrite(inb2, b2); analogWrite(pwb, pb);
      // reset time interval timer
      t.reset();
      // update output state
      P = 1;
    }

  }else{

    // motor state is expected to be powered off
    // but the power output is still on -> switch off
    if(P == 1){
      // set all polarities and powers to zero
      analogWrite(pwa, 0); digitalWrite(ina1, LOW); digitalWrite(ina2, LOW);
      analogWrite(pwb, 0); digitalWrite(inb1, LOW); digitalWrite(inb2, LOW);
      // update output state
      P = 0;
    }
  }
  return 0;
}
