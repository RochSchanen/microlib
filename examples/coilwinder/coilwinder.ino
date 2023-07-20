//  coilwinder.ino

#include <microlib.h>

// todo:
// continuous selection of the wire size with stop
// adjust spool substep value
// adjust spool min and max rpm

////////////////////////
//// GENERAL MACROS ////
////////////////////////

// clear bit
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// set bit
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |=  _BV(bit))
#endif

///////////////////////////
//// GENERAL FUNCTIONS ////
///////////////////////////

// general string copy function
void strcpy(char *dest, char *sour){
    uint8_t i;
    i = 0;
    while(*(sour+i)>'\0') *(dest+i)=*(sour+i);
    *(dest+i)='\0';
    return;}

/////////////////////////
//// PIN DEFINITIONS ////
/////////////////////////

// SM0: STEPPER MOTOR 0 (bottom connector)
// PWMs are pins 2 and 3 (timer counter #3)
// digital pins are 22, 23, 24, 25 (port A)
// coil A: 3, 22, 23 (PWM, +, -)
// coil B: 2, 24, 25 (PWM, +, -)
#define SM0_PWMA 3
#define SM0_INA1 22
#define SM0_INA2 23
#define SM0_PWMB 2
#define SM0_INB1 24
#define SM0_INB2 25

// SM1: STEPPER MOTOR 1 (middle connector)
// PWMs are pins 6 and 7 (timer counter #4)
// digital pins are 26, 27, 28, 29 (port A)
// coil A: 7, 26, 27 (PWM, +, -)
// coil B: 6, 28, 29 (PWM, +, -)
#define SM1_PWMA 7
#define SM1_INA1 26
#define SM1_INA2 27
#define SM1_PWMB 6
#define SM1_INB1 28
#define SM1_INB2 29

// SM2: STEPPER MOTOR 2 (top connector)
// PWMs are pins 44 and 45 (timer counter #5)
// digital pins are 30, 31, 32, 33 (port C)
// coil A: 44, 30, 31 (PWM, +, -)
// coil B: 45, 32, 33 (PWM, +, -)
#define SM2_PWMA 44
#define SM2_INA1 30
#define SM2_INA2 31
#define SM2_PWMB 45
#define SM2_INB1 32
#define SM2_INB2 33

// IE0: INCREMENTAL ENCODER 0 (bottom connector)
// A: 40, B: 41 (port G bit 0, 1)
// I: 14 (port J bit 0)
#define IE0_A 40
#define IE0_B 41
#define IE0_I 14

// IE1: INCREMENTAL ENCODER 1 (top connector)
// A: 48, B: 49 (port L bit 0, 1)
// I: 15 (port J bit 1)
#define IE1_A 48
#define IE1_B 49
#define IE1_I 15

// IE2: INCREMENTAL ENCODER 2 (handheld box)
// A: 16, B: 17 (port H bit 0, 1)
// I: 39 (port G bit 2)
#define IE2_A 16
#define IE2_B 17
#define IE2_I 39

// BT: BUTTONS
// RE: 34, DR: 35, UP: 36, DN: 37 (port C bit 0, 1, 2, 3)
#define BT_RE 34 // Front Push Button (emergency stop)
#define BT_DR 35 // Handheld Box Toggle Switch "on-none-on"
#define BT_UP 36 // Front Toggle Switch "(on)-off-(on)"
#define BT_DN 37 // Front Toggle Switch "(on)-off-(on)"

/////////////////////////////
//// DEVICES DECLARATION ////
/////////////////////////////

// stepper motors instance
microlib_StM sm_coil;
microlib_StM sm_carriage;
microlib_StM sm_spool;

// incremental encoders and trigger signals
microlib_IEc ie_knob;  microlib_Btn bt_kn; // knob stop
microlib_IEc ie_spool; microlib_Btn bt_sr; // spool zero reset
microlib_IEc ie_coil;  microlib_Btn bt_cr; // full turn trigger

// buttons and switch
microlib_Btn bt_up; // toggle switch select up
microlib_Btn bt_dn; // toggle switch select down
microlib_Btn bt_dr; // toggle switch select direction
microlib_Btn bt_al; // push button emergency stop (alarm)

// Solomon Systech Diplay
microlib_SSD1306 dsp = microlib_SSD1306();

/////////////////
//// DISPLAY ////
/////////////////

// bus
microlib_TWi bus = microlib_TWi();

// display buffers
microlib_DSB db_counter;        // counter value
microlib_DSB db_position;       // cariage position value
microlib_DSB db_wiresize;       // wire size value
microlib_DSB db_rpm;            // coil former rotation speed
microlib_DSB db_counter_stat;   // counter state indicator (step or continuous)
microlib_DSB db_position_stat;  // cariage state indicator (step or continuous)
microlib_DSB db_wiresize_stat;  // wire size state indicator (step selected)
microlib_DSB db_dir;            // direction state indicator
microlib_DSB db_alarm;          // alarm raised state indicator

// dislay strings
char sb_counter[]  = "+@@@@@@.@";
char sb_position[] = "+@@@@@@.@";
char sb_wiresize[] =     "@@@.@";
char sb_rpm[]      =     "+@@.@";
char sb_counter_stat[]  = "  ";
char sb_position_stat[] = "  ";
char sb_wiresize_stat[] = "  ";
char sb_alarm[]         = "AB";
char sb_dir[]           = " >";
char sb_tmp[]           = "   ";  // permanent text

// clear function
void clear(void){
    char clrstr[]={' ','\0'};
    for(uint8_t Y=0; Y<4; Y++)
        for(uint8_t X=0; X<16; X++){
        clrstr[0]=' '; dsp.putString(X, Y, clrstr);
        while(!dsp.ready()){bus.updt(); dsp.updt();}
        }
    return;}

/////////////////////////
//// STATE VARIABLES ////
/////////////////////////

int16_t ie_knob_v;      // knob value  [steps]
int16_t ie_spool_v;     // spool value [steps]
int16_t ie_coil_v;      // coil value  [steps]
int32_t sm_carriage_v;  // carriage motor position value  [sub steps]
int32_t sm_coil_v;      // coil motor position value      [sub steps]
int32_t interval_v;     // interval value
int8_t dir_v;           // direction value
uint8_t alarm_f;        // spool flag
uint16_t wire_v;        // wire size value    [0.1 um]
float Counter_o;        // counter offset     [sub steps]
float Position_o;       // position offset    [micrometers]
float Counter_ref;      // counter reference  [sub steps]
float Position_ref;     // position reference [micrometers]
float Position_target;  //                    [sub steps]
uint32_t CurrentState;  // Current Machine State
uint32_t InitState;     // Init trigger flag

//////////////////////////////////////////////////
//// HARDWARE SPECIFIC DEFINITIONS AND MACROS ////
//////////////////////////////////////////////////

// 16  substeps per step, 200 steps per turn, 5mm per turn
// 1m = 100cm = 1000mm = 200 turns = 40 000 steps = 640 000 substeps
// 1mm =  40 steps = 640 substeps
// 1um =   0.4 steps = 0.64 substeps
// 1 substep = 0.0625 step = 0.0003125 turn = 1.5625 um
// 0.1 turn = 20 steps = 320 substeps = 2^6*5 substeps

#define SUBSTEPSIZE 2
#define SPOOL_MAX 28
#define SPOOL_MIN 2
#define WIRE(x) ((float)x / 10.0)
#define COUNTER(x) ((float)x / 200.0 / 16.0)
// #define COUNTER(x) ((float)x / 500.0)
#define POSITION(x) ((float)x * 5000.0 / 200.0 / 16.0)
#define RPM(x)((float)(1000000L * 60 * SUBSTEPSIZE / 16 / 200) / (float)x )
#define SPOOL(x) (1800.0 * (float)(SPOOL_MAX - SPOOL_MIN) / ((float)abs(x) - (float)SPOOL_MIN))

/////////////////////
//// ALARM CODES ////
/////////////////////

#define ALARM_SPOL (1<<0) // SPOOL UNREFERENCED
#define ALARM_USER (1<<1) // USER TRIGGERED
#define ALARM_ARML (1<<2) // ARM TOO LOW
#define ALARM_ARMH (1<<3) // ARM TOO HIGH
#define ALARM_CARL (1<<4) // CARRIAGE LEFT LIMIT REACHED
#define ALARM_CARR (1<<5) // CARRIAGE RIGHT LIMIT REACHED

/////////////////////////////
//// MACHINE STATE CODES ////
/////////////////////////////

#define STATE_ALARM     0
#define STATE_COIL_WIND 1
#define STATE_COIL_ONLY 2
#define STATE_COIL_STEP 3
#define STATE_CARR_ONLY 4
#define STATE_CARR_STEP 5
#define STATE_WIRE_STEP 6

///////////////
//// SETUP ////
///////////////

void setup() {

    // Serial.begin(9600);
    // Serial.write("starting...\n");

    pinMode(IE0_A, INPUT_PULLUP);
    pinMode(IE0_B, INPUT_PULLUP);
    ie_spool.setup(1, 0, PING); // to generalise
    ie_spool.set(0, 0, 1);

    pinMode(IE1_A, INPUT_PULLUP);
    pinMode(IE1_B, INPUT_PULLUP);
    ie_coil.setup(1, 0, PINL);  // to generalise
    ie_coil.set(0, 0, 1);

    // hand held box
    pinMode(IE2_A, INPUT_PULLUP);
    pinMode(IE2_B, INPUT_PULLUP);
    ie_knob.setup(1, 0, PINH);  // to generalise
    ie_knob.set(0, 0, 1);

    bt_al.setup(BT_RE); // alarm
    bt_up.setup(BT_UP); // select up
    bt_dn.setup(BT_DN); // select down
    bt_dr.setup(BT_DR); // direction
    bt_cr.setup(IE0_I); // counter reset
    bt_sr.setup(IE1_I); // spool reset
    bt_kn.setup(IE2_I); // knob reset

    cbi(TCCR3B, CS31); // increase pwm frequency: counter #3
    cbi(TCCR4B, CS41); // increase pwm frequency: counter #4
    cbi(TCCR5B, CS51); // increase pwm frequency: counter #5

    sm_spool.setup(     SM0_PWMA, SM0_INA1, SM0_INA2,
                        SM0_PWMB, SM0_INB1, SM0_INB2);
    sm_carriage.setup(  SM1_PWMA, SM1_INA2, SM1_INA1,
                        SM1_PWMB, SM1_INB1, SM1_INB2);
    sm_coil.setup(      SM2_PWMA, SM2_INA1, SM2_INA2,
                        SM2_PWMB, SM2_INB1, SM2_INB2);

    dsp.setup(&bus, 0x3C);

    clear();

    db_position_stat.setup(&dsp, sb_position_stat, 0, 1);
    db_wiresize_stat.setup(&dsp, sb_wiresize_stat, 0, 2);
    db_counter_stat.setup(&dsp, sb_counter_stat, 0, 0);
    db_position.setup(&dsp, sb_position, 2, 1);
    db_wiresize.setup(&dsp, sb_wiresize, 6, 2);
    db_counter.setup(&dsp, sb_counter, 2, 0);
    db_alarm.setup(&dsp, sb_alarm, 0, 3);
    db_rpm.setup(&dsp, sb_rpm, 6, 3);
    db_dir.setup(&dsp, sb_dir, 3, 3);

    sm_carriage_v = sm_carriage.getpos();
    sm_coil_v = sm_coil.getpos();

    ie_spool_v = ie_spool.get();
    ie_knob_v = ie_knob.get();
    ie_coil_v = ie_coil.get();

    ie_spool.set(0, -28, +28);
    ie_coil.set(0, -(1L<<30), +(1L<<30));

    dir_v = +1;
    wire_v = 2500;
    Counter_o = 0;
    Position_o = 0;
    Counter_ref = COUNTER(sm_coil_v);
    Position_ref = POSITION(sm_carriage_v);
    alarm_f = ALARM_SPOL;
    CurrentState = STATE_ALARM;
    InitState = 1;

    dtostrf(COUNTER(sm_coil_v) - Counter_o, 9, 1, sb_counter);
    dtostrf(POSITION(sm_carriage_v) - Position_o, 9, 1, sb_position);
    dtostrf(WIRE(wire_v), 5, 1, sb_wiresize);
    dtostrf(0.0, 5, 1, sb_rpm);

    strcpy(sb_tmp," ~m"); dsp.putString(11, 1, sb_tmp);
    while(!dsp.ready()){bus.updt(); dsp.updt();}
    strcpy(sb_tmp," ~m"); dsp.putString(11, 2, sb_tmp);
    while(!dsp.ready()){bus.updt(); dsp.updt();}
    strcpy(sb_tmp,"rpm"); dsp.putString(12, 3, sb_tmp);
    while(!dsp.ready()){bus.updt(); dsp.updt();}

    return;}

uint8_t SelectorUpdate(int8_t Current){
    if(bt_up.pressed()) Current -= 1;
    if(bt_dn.pressed()) Current += 1;
    if(Current < 1) Current = 6;
    if(Current > 6) Current = 1;
    if(alarm_f) Current = STATE_ALARM;
    return Current;}

void loop(){

    int32_t Current_Value;

    /********** update I/O **********/

        /*** display ***/
        db_wiresize_stat.updt();
        db_position_stat.updt();
        db_counter_stat.updt();
        db_position.updt();
        db_wiresize.updt();
        db_counter.updt();
        db_rpm.updt();
        db_alarm.updt();
        db_dir.updt();
        dsp.updt();
        bus.updt();

        /*** buttons ***/
        bt_kn.updt();
        bt_up.updt();
        bt_dn.updt();
        bt_dr.updt();
        bt_sr.updt();
        bt_al.updt();
        bt_cr.updt();

        /*** incremental encoders ***/
        ie_spool.updt(PINL);    // to generalise
        ie_knob.updt(PINH);     // to generalise
        ie_coil.updt(PING);     // to generalise

        /*** motors ***/
        sm_carriage.updt();
        sm_spool.updt();
        sm_coil.updt();

    /********** update machine state **********/

    switch(CurrentState){

        case STATE_ALARM:

            if(InitState){

                // Serial.write("Init ALARM\n");

                sm_coil.release();
                sm_spool.release();
                sm_carriage.release();

                strcpy(sb_alarm, "AB");

                InitState = 0;}

            if(alarm_f == 0) CurrentState = STATE_COIL_WIND;

            if(CurrentState == STATE_ALARM){

                // remove alarm flags when
                // signalled ok by the hardware

                if(bt_sr.pressed()){
                    sm_spool.hold();
                    ie_spool.set(0);
                    alarm_f &= (~ALARM_SPOL);}

            }else{

                // Serial.write("Exit ALARM\n");

                strcpy(sb_alarm, "  ");
                InitState = 1;}

            break;

        case STATE_COIL_WIND:

            if(InitState){

                // Serial.write("Init WIND\n");

                ie_knob.set(0, -24, +24);
                ie_knob_v = ie_knob.get();
                Counter_ref = COUNTER(sm_coil.getpos());
                Position_ref = POSITION(sm_carriage.getpos());
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_COIL_WIND){

                // update RPM
                Current_Value = ie_knob.get();
                if(ie_knob_v != Current_Value){
                    if(Current_Value == 0){
                        sm_coil.hold();
                        dtostrf(0.0, 5, 1, sb_rpm);
                    }else{
                        interval_v = 1000.0*24.0/(float)abs(Current_Value);
                        sm_coil.interval(interval_v, SUBSTEPSIZE);
                        if(Current_Value > 0) sm_coil.forward();
                        if(Current_Value < 0) sm_coil.reverse();
                        if(Current_Value < 0) interval_v = -interval_v;
                        dtostrf(RPM(interval_v), 5, 1, sb_rpm);}
                    ie_knob_v = Current_Value;}

                // update CARRIAGE
                sm_carriage.target(
                    (int32_t)
                        (((COUNTER(sm_coil.getpos())
                            - Counter_ref) * (float) wire_v / 10.0 * dir_v
                                + Position_ref) * 16 * 200 / 5000));

                // stop if pressed
                if(bt_kn.pressed()) ie_knob.set(0);

            }else{

                // Serial.write("Exit WIND\n");

                sm_coil.hold();
                dtostrf(0.0, 5, 1, sb_rpm);
                InitState = 1;}

            break;

        case STATE_COIL_ONLY:

            if(InitState){
                strcpy(sb_counter_stat, "{}");
                ie_knob.set(0, -12, +12);
                ie_knob_v = ie_knob.get();
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_COIL_ONLY){

                Current_Value = ie_knob.get();
                if(ie_knob_v != Current_Value){
                    if(Current_Value == 0){
                        sm_coil.hold();
                        dtostrf(0.0, 5, 1, sb_rpm);
                    }else{
                        interval_v = 1000.0*12.0/(float)abs(Current_Value);
                        sm_coil.interval(interval_v, SUBSTEPSIZE);
                        if(Current_Value > 0) sm_coil.forward();
                        if(Current_Value < 0) sm_coil.reverse();}
                    ie_knob_v = Current_Value;}
                if(bt_kn.pressed()) ie_knob.set(0);

            }else{

                sm_coil.hold();
                strcpy(sb_counter_stat, "  ");
                InitState = 1;}

            break;

        case STATE_CARR_ONLY:

            if(InitState){
                strcpy(sb_position_stat, "{}");
                ie_knob.set(0, -12, +12);
                ie_knob_v = ie_knob.get();
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_CARR_ONLY){
                Current_Value = ie_knob.get();
                if(ie_knob_v != Current_Value){
                    if(Current_Value == 0){
                        sm_carriage.hold();
                        dtostrf(0.0, 5, 1, sb_rpm);
                    }else{
                        interval_v = 1000.0*12.0/(float)abs(Current_Value);
                        sm_carriage.interval(interval_v, SUBSTEPSIZE);
                        if(Current_Value > 0) sm_carriage.forward();
                        if(Current_Value < 0) sm_carriage.reverse();}
                    ie_knob_v = Current_Value;}
                if(bt_kn.pressed()) ie_knob.set(0);

            }else{

                sm_carriage.hold();
                strcpy(sb_position_stat, "  ");
                InitState = 1;}

            break;

        case STATE_COIL_STEP:

            if(InitState){
                strcpy(sb_counter_stat, "[]");
                ie_knob.set(0, -12, +12);
                sm_coil.interval(1000, 4);
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_COIL_STEP){

                Current_Value = ie_knob.get();
                if(Current_Value != 0){
                    sm_coil.shift(Current_Value*4);
                    ie_knob.set(0);}
                if(bt_kn.pressed()){
                    Counter_o = COUNTER(sm_coil.getpos());
                    dtostrf(COUNTER(sm_coil.getpos())-Counter_o, 9, 1, sb_counter);}

            }else{

                strcpy(sb_counter_stat, "  ");
                InitState = 1;}

            break;

        case STATE_CARR_STEP:

            if(InitState){
                strcpy(sb_position_stat, "[]");
                ie_knob.set(0, -12, +12);
                sm_carriage.interval(1000, 4);
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_CARR_STEP){
                Current_Value = ie_knob.get();
                if(Current_Value != 0){
                    sm_carriage.shift(Current_Value*4);
                    ie_knob.set(0);}
                if(bt_kn.pressed()){
                    Position_o = POSITION(sm_carriage.getpos());
                    dtostrf(POSITION(sm_carriage.getpos())-Position_o, 9, 1, sb_position);}

            }else{

                strcpy(sb_position_stat, "  ");
                InitState = 1;}

            break;

        case STATE_WIRE_STEP:

            if(InitState){
                strcpy(sb_wiresize_stat, "[]");
                ie_knob.set(wire_v, 0, 10000);
                InitState = 0;}

            CurrentState = SelectorUpdate(CurrentState);

            if(CurrentState == STATE_WIRE_STEP){
                Current_Value = ie_knob.get();
                if(wire_v != Current_Value){
                    dtostrf(WIRE(Current_Value), 5, 1, sb_wiresize);
                    wire_v = Current_Value;}

            }else{

                strcpy(sb_wiresize_stat, "  ");
                InitState = 1;}

            break;
        }

    if(alarm_f == 0){
        bt_sr.pressed(); // clear button
        // if(bt_sr.pressed()) ie_spool.set(0); // re-set arm's origin
        Current_Value = ie_spool.get();
        if(ie_spool_v != Current_Value){
            interval_v = 0;
            if(Current_Value > +SPOOL_MIN){
                interval_v = SPOOL(+Current_Value);
                sm_spool.forward();}
            if(Current_Value < -SPOOL_MIN){
                interval_v = SPOOL(-Current_Value);
                sm_spool.reverse();}
            if(interval_v) sm_spool.interval(interval_v, 8);
            else sm_spool.hold();
            ie_spool_v = Current_Value;}}

    Current_Value = sm_coil.getpos();
    if(sm_coil_v != Current_Value){
        dtostrf(COUNTER(Current_Value)-Counter_o, 9, 1, sb_counter);
        sm_coil_v = Current_Value;}

    // Current_Value = ie_coil.get();
    // if(ie_coil_v != Current_Value){
    //  dtostrf(COUNTER(Current_Value), 9, 1, sb_counter);
    //  ie_coil_v = Current_Value;}

    Current_Value = sm_carriage.getpos();
    if(sm_carriage_v != Current_Value){
        dtostrf(POSITION(Current_Value)-Position_o, 9, 1, sb_position);
        sm_carriage_v = Current_Value;}

    if(bt_dr.pressed()){
        Counter_ref = COUNTER(sm_coil.getpos());
        Position_ref = POSITION(sm_carriage.getpos());
        strcpy(sb_dir, "< ");
        dir_v = -1;}

    if(bt_dr.released()){
        Counter_ref = COUNTER(sm_coil.getpos());
        Position_ref = POSITION(sm_carriage.getpos());
        strcpy(sb_dir, " >");
        dir_v = +1;}

    if(bt_al.pressed()) alarm_f |= ALARM_SPOL;

}
