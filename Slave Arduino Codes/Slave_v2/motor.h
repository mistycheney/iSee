/*
 * motor.h
 *
 *  Created on: Jun 16, 2012
 *      Author: yuncong
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <MatrixMath.h>
#include <digitalWriteFast.h>
#include "datatypes.h"

#include "I2CSlave.h"

// pin 0,1 are for upload should not connect, pin 4,5 reserved for I2C
#define PINA1 A0
#define PINA2 A1
#define DIRA 12
#define PWMA 3

#define PINB1 A2
#define PINB2 A3
#define DIRB 13
#define PWMB 11

#define PINC1 A4
#define PINC2 A5
#define DIRC 8
#define PWMC 10

#define PIND1 2
#define PIND2 9
#define DIRD 7
#define PWMD 6

void motor_setup();
void funcA1();
void funcA2();
void funcB1();
void funcB2();
void funcC1();
void funcC2();
void funcD1();
void funcD2();
void motor_go();
void Drive();
void compute_rotation_matrix(float angle);
void compute_PWM_DIR();
void update_state();
void BezierInterp();
void CircularObservation();
void RecoveryMode();

//extern const uint8_t ENDPATH_BIT; // has the robot just finished a path?
//extern const uint8_t FINSEG_BIT; // has the robot just finished a segment?
//extern const uint8_t CIRCOBS_BIT; // Alternative 1. should the robot do circular observation at the end of path?


#endif /* MOTOR_H_ */
