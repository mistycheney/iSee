// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef MotorTest_H_
#define MotorTest_H_
#include "Arduino.h"
//add your includes for the project MotorTest here

#include <digitalWriteFast.h>
//#define NO_PORTB_PINCHANGES //to indicate that port b will not be used for pin change interrupts using version 1.6beta
#include <PinChangeInt.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project MotorTest here

// pin 0,1 are for upload should not connect, pin A4,A5 reserved for I2C
// RuggedCircuits motor board uses 3(PWM1), 12(DIR1), 11(PWM2), 13(DIR2)
// Empirically, pin 9 can not be used as digital pin, only use as PWM
// encoder increase if the wheel rotate clock-wise
#define PINA1 A0 //white
#define PINA2 A1 //yellow
#define DIRA 12
#define PWMA 3

#define PINB1 A2
#define PINB2 A3
#define DIRB 13
#define PWMB 11

#define PINC1 2
#define PINC2 10
#define DIRC 8
#define PWMC 9

#define PIND1 4
#define PIND2 5
#define DIRD 7
#define PWMD 6

void funcA1();
void funcA2();
void funcB1();
void funcB2();
void funcC1();
void funcC2();
void funcD1();
void funcD2();

//Do not add code below this line
#endif /* MotorTest_H_ */
