/*
 * motor.cpp
 *
 *  Created on: Jun 18, 2012
 *      Author: Edoardo
 */

#include "motor.h"

//#define NO_PORTB_PINCHANGES //to indicate that port b will not be used for pin change interrupts using version 1.6beta
#include <PinChangeInt.h>

const uint8_t ENDPATH_BIT = 1;
const uint8_t FINSEG_BIT = 2;
const uint8_t CIRCOBS_BIT = 4;
const uint8_t NEWDATA_BIT = 8;

MatrixMath math;

//inline void serial_printf(fstr, float* data, int num) {
//	char* ref_str = (char*) malloc(30);
//	sprintf(ref_str, "%f, %f, %f, %f, %f, %f", setpoint0[curr_sp],
//			setpoint1[curr_sp], setpoint2[curr_sp], setpoint3[curr_sp], setpoint4[curr_sp],
//			setpoint5[curr_sp], reference[6]);
//	Serial.println(ref_str);
//	free(ref_str);
//}

float waypointX[4];
float waypointY[4];

extern master_data md;
extern slave_data sd;

//Encoder Positions
double countA = 0, countB = 0, countC = 0, countD = 0;

float Rot[4][4];
//Bezier
const float BCoeff[11][4] = { { 1.0000, 0, 		0, 		0 },
							  { 0.7290, 0.2430, 0.0270, 0.0010 },
							  { 0.5120, 0.3840, 0.0960, 0.0080 },
							  { 0.3430, 0.4410, 0.1890, 0.0270 },
							  { 0.2160, 0.4320, 0.2880, 0.0640 },
							  { 0.1250, 0.3750, 0.3750, 0.1250 },
							  { 0.0640, 0.2880, 0.4320, 0.2160 },
							  { 0.0270, 0.1890, 0.4410, 0.3430 },
							  { 0.0080, 0.0960, 0.3840, 0.5120 },
							  { 0.0010, 0.0270, 0.2430, 0.7290 },
							  { 0, 		0,	 	0, 		1.0000 }};
float Vref = 0.3;

//LQR control vector
float state[6] = { 0 };
float reference[6] = { 0 };
// 10 setpoints for each segment, each setpoint contains (x,y,t,vx,vy,vt)
float setpoint0[11], setpoint1[11], setpoint2[11], setpoint3[11], setpoint4[11],
		setpoint5[11];
float input[4];
int torquePWM[4];
boolean direction[4];

unsigned long startTime = 0, lastTime = 0;

//coming from MatLab LQR toolbox, precalculated we can even have various K for different type of navigation i.e: with different weight on Q and R//

const float K[4][6] = { { 	0.0, 	1.0825, 0.4841,  0.0, 	 1.0479, 0.1712 },
						{ -1.0825, 	0.0, 	0.4841, -1.0479, 0.0, 	 0.1712 },
						{  0.0,    -1.0825, 0.4841,  0.0,	-1.0479, 0.1712 },
						{  1.0825,  0.0,  	0.4841,  1.0479, 0.0, 	 0.1712 }};

const float qTOv[3][4] = {  {0, 	 -0.5000, 0, 	  0.5000 },
							{0.5000, -0.0000,-0.5000, 0.0000 },
							{1.6667,  1.6667, 1.6667, 1.6667 } };

boolean Recovery = 0;

//PIN A
inline void funcA1() {
	PCintPort::pinState != digitalReadFast(PINA2) ? countA++ : countA--;
}
inline void funcA2() {
	PCintPort::pinState != digitalReadFast(PINA1) ? countA-- : countA++;
}
//PIN B
inline void funcB1() {
	PCintPort::pinState != digitalReadFast(PINB2) ? countB++ : countB--;
}
inline void funcB2() {
	PCintPort::pinState != digitalReadFast(PINB1) ? countB-- : countB++;
}
//PIN C
inline void funcC1() {
	PCintPort::pinState != digitalReadFast(PINC2) ? countC++ : countC--;
}
inline void funcC2() {
	PCintPort::pinState != digitalReadFast(PINC1) ? countC-- : countC++;
}
//PIN D
inline void funcD1() {
	PCintPort::pinState != digitalReadFast(PIND2) ? countD++ : countD--;
}
inline void funcD2() {
	PCintPort::pinState != digitalReadFast(PIND1) ? countD-- : countD++;
}

void motor_setup() {
	pinMode(PINA1, INPUT);
	digitalWrite(PINA1, HIGH);
	PCintPort::attachInterrupt(PINA1, &funcA1, CHANGE);
	pinMode(PINA2, INPUT);
	digitalWrite(PINA2, HIGH);
	PCintPort::attachInterrupt(PINA2, &funcA2, CHANGE);
	pinMode(DIRA, OUTPUT);
	digitalWrite(DIRA, LOW);
	pinMode(PWMA, OUTPUT);
	digitalWrite(PWMA, LOW);

	pinMode(PINB1, INPUT);
	digitalWrite(PINB1, HIGH);
	PCintPort::attachInterrupt(PINB1, &funcB1, CHANGE);
	pinMode(PINB2, INPUT);
	digitalWrite(PINB2, HIGH);
	PCintPort::attachInterrupt(PINB2, &funcB2, CHANGE);
	pinMode(DIRB, OUTPUT);
	digitalWrite(DIRB, LOW);
	pinMode(PWMB, OUTPUT);
	digitalWrite(PWMB, LOW);

	pinMode(PINC1, INPUT);
	digitalWrite(PINC1, HIGH);
	PCintPort::attachInterrupt(PINC1, &funcC1, CHANGE);
	pinMode(PINC2, INPUT);
	digitalWrite(PINC2, HIGH);
	PCintPort::attachInterrupt(PINC2, &funcC2, CHANGE);
	pinMode(DIRC, OUTPUT);
	digitalWrite(DIRC, LOW);
	pinMode(PWMC, OUTPUT);
	digitalWrite(PWMC, LOW);

	pinMode(PIND1, INPUT);
	digitalWrite(PIND1, HIGH);
	PCintPort::attachInterrupt(PIND1, &funcD1, CHANGE);
	pinMode(PIND2, INPUT);
	digitalWrite(PIND2, HIGH);
	PCintPort::attachInterrupt(PIND2, &funcD2, CHANGE);
	pinMode(DIRD, OUTPUT);
	digitalWrite(DIRD, LOW);
	pinMode(PWMD, OUTPUT);
	digitalWrite(PWMD, LOW);
}

/**
 * The loop that runs the motor
 */
void motor_go() {

	sd.status |= FINSEG_BIT;
	if ((sd.status & ENDPATH_BIT) && (sd.status & CIRCOBS_BIT)){
		while (~(sd.status & CIRCOBS_BIT)){}
	}
	// if at the end of the entire path, do circular observation
	if (sd.status & ENDPATH_BIT) {
		Serial.println("path end.");
		sd.status &= ~FINSEG_BIT;
		sd.status &= ~ENDPATH_BIT;
		sd.status |= CIRCOBS_BIT; //This means we have seen that's the end of the path and we're going for a circular rotation
		CircularObservation();
	}

	// if at the end of segment, set new waypoints;
	// the next 3 waypoints should already be received in sd
	if (~(sd.status & CIRCOBS_BIT)) { //this means that if we are not going for a circular rotation we ask for 3 new waypoints
		I2Csend();
		I2Creceive();
		sd.status &= ~NEWDATA_BIT;

		waypointX[0] = state[0];
		waypointY[0] = state[1];
		memcpy(&(waypointX[1]), sd.waypointX, 3 * sizeof(float));
		memcpy(&(waypointY[1]), sd.waypointY, 3 * sizeof(float));
		Serial.print("Waypoint");
		Serial.print(waypointX[1]);
		Serial.print(waypointY[1]);
		Serial.print(waypointX[2]);
		Serial.print(waypointY[2]);
		Serial.print(waypointX[3]);
		Serial.print(waypointY[3]);
		BezierInterp();
		sd.status &= ~FINSEG_BIT;
	}
	for (int t=1; t<=10; t= t+1){
		Serial.print("setpoint n");
		Serial.print(t);
		Serial.print(": ");
		Serial.print(setpoint0[t]);
		Serial.print(" ");
		Serial.print(setpoint1[t]);
		Serial.print(" ");
		Serial.print(setpoint2[t]);
		Serial.print(" ");
		Serial.print(setpoint3[t]);
		Serial.print(" ");
		Serial.print(setpoint4[t]);
		Serial.print(" ");
		Serial.println(setpoint5[t]);}

	Drive();
}

int curr_sp = 1; // current target setpoint
/**
 * Drive through next 3 waypoints / 10 setpoints
 */
void Drive() {

	while (~(sd.status & FINSEG_BIT)) {

		// compute motor PWM and direction
		float Kstate[4], Ksetpoint[4], setpoint[6], u[4];
		setpoint[0] = setpoint0[curr_sp];
		setpoint[1] = setpoint1[curr_sp];
		setpoint[2] = setpoint2[curr_sp];
		setpoint[3] = setpoint3[curr_sp];
		setpoint[4] = setpoint4[curr_sp];
		setpoint[5] = setpoint5[curr_sp];
		//math.MatrixMult((float*) K, (float*) uu, 4, 6, 1, (float*) u);
		math.MatrixMult((float*)K,(float*)state, 4, 6, 1,(float*)Kstate);//why was not declared?? It's a function!
		math.MatrixMult((float*)K,(float*)setpoint, 4, 6, 1,(float*) Ksetpoint);
		u[0]=(Ksetpoint[0]-Kstate[0]);
		u[1]=(Ksetpoint[1]-Kstate[1]);
		u[2]=(Ksetpoint[2]-Kstate[2]);
		u[3]=(Ksetpoint[3]-Kstate[3]);
		compute_rotation_matrix(state[2]);
		math.MatrixMult((float*) Rot, (float*) u, 4, 4, 1, (float*) input);
		compute_PWM_DIR();

		// write signal to motor pins
		analogWrite(PWMA, torquePWM[0]);
		digitalWrite(DIRA, direction[0]);
		analogWrite(PWMB, torquePWM[1]);
		digitalWrite(DIRB, direction[1]);
		analogWrite(PWMC, torquePWM[2]);
		digitalWrite(DIRC, direction[2]);
		analogWrite(PWMD, torquePWM[3]);
		digitalWrite(DIRD, direction[3]);

		// update state
		update_state();

//		md.x = state[0];
//		md.y = state[1];
//		md.theta = state[2];
//		md.status = sd.status;
//		I2Csend();

		// too far from reference
		if (abs(state[0]-setpoint0[curr_sp]) > 0.3
				&& abs(state[1]-setpoint1[curr_sp]) > 0.3 && Recovery == 0) {
			Serial.println("RECOVERY");
			analogWrite(PWMA, 0);
			analogWrite(PWMB, 0);
			analogWrite(PWMC, 0);
			analogWrite(PWMD, 0);
			Recovery = 1;
			RecoveryMode();
		}

		// if close to setpoint, proceed to the next one
		if ((abs(state[0]-setpoint0[curr_sp]) < 0.05
				&& abs(state[1]-setpoint1[curr_sp]) < 0.05
				&& ~(sd.status & CIRCOBS_BIT))
				|| (abs(state[2]-setpoint2[curr_sp]) < 0.03
						&& (sd.status & CIRCOBS_BIT))) {
			Recovery = 0;

			if (curr_sp == 10) {
				analogWrite(PWMA, 0);
				analogWrite(PWMB, 0);
				analogWrite(PWMC, 0);
				analogWrite(PWMD, 0);
				sd.status |= FINSEG_BIT;
				//if ( sd.status & CIRCOBS_BIT) { //i'm taking this off only for testing purpose//
				//	sd.status &= ~CIRCOBS_BIT;}
			}

			Serial.print("finished setpoint ");
			Serial.println(curr_sp);
			curr_sp++;

			Serial.print("setpoint ");
			Serial.print(curr_sp);
			Serial.print(": ");
			Serial.print(setpoint0[curr_sp]);
			Serial.print(" ");
			Serial.print(setpoint1[curr_sp]);
			Serial.print(" ");
			Serial.print(setpoint2[curr_sp]);
			Serial.print(" ");
			Serial.print(setpoint3[curr_sp]);
			Serial.print(" ");
			Serial.print(setpoint4[curr_sp]);
			Serial.print(" ");
			Serial.println(setpoint5[curr_sp]);
		}
		delay(5);
	}
}

void compute_rotation_matrix(float angle) {
	double coth = cos(angle);
	double sith = sin(angle);
	Rot[0][0] = (1 + coth) / 2;
	Rot[0][1] = sith / 2;
	Rot[0][2] = (1 - coth) / 2;
	Rot[0][3] = -Rot[0][1];
	Rot[1][0] = Rot[0][3];
	memcpy(&Rot[1][1], &Rot[0][0], 3 * sizeof(double));
	memcpy(&Rot[2][0], &Rot[0][2], 2 * sizeof(double));
	memcpy(&Rot[2][2], &Rot[0][0], 2 * sizeof(double));
	memcpy(&Rot[3][0], &Rot[0][1], 3 * sizeof(double));
	Rot[3][3] = Rot[0][0];
}

/*
 * Convert input number to PWM and DIR that are fed to the motors
 */
void compute_PWM_DIR() {
	const float upperlimit = 0.8;
	const float lowerlimit = 0.011;
	float absInput;
	float range = upperlimit - lowerlimit;
	Serial.print("torquePWM[:]");
	for (uint8_t i = 0; i <= 3; i++) {
		if (input[i] >= 0) {
			direction[i] = HIGH;
		} else {
			direction[i] = LOW;
		}
		absInput=abs(input[i]);
		if (absInput>upperlimit){
			input[i]=upperlimit;
		}

		torquePWM[i] = (absInput > lowerlimit) ? (absInput / range) * 45 + 39 : 0; //this was wrong! in this way all the negative input are cut down!! NEED ABS (was 40 +36)
		Serial.println(torquePWM[i]);
	}
}

/**
 * convert encoder readings to global velocity, and update the state
 */
void update_state() {
	float vlocal[3], vglobal[3];
	float dX = 0, dY = 0, dth = 0;

	// check the time to move through last update
	startTime = millis();
	float deltaT = startTime - lastTime; //SHOULD BE CONVERTED IN SEC//
	lastTime = startTime;

	// convert encoder readings to wheel speed
	float wheelspeed[4];
	float coeff = deltaT * 9.6;
	wheelspeed[0] = countA / coeff; // m/s in theory
	wheelspeed[1] = countB / coeff;
	wheelspeed[2] = countC / coeff;
	wheelspeed[3] = countD / coeff;
	//Serial.print("wheelcount [:]");
	//Serial.println(countA);
	//Serial.println(countB);
	//Serial.println(countC);
	//Serial.println(countD);

	//Serial.print("wheelspeed [:] ");
	//Serial.println(wheelspeed[0], 6);
	//Serial.println(wheelspeed[1], 6);
	//Serial.println(wheelspeed[2], 6);
	//Serial.println(wheelspeed[3], 6);

	// reset encoder counts
	countA = 0;
	countB = 0;
	countC = 0;
	countD = 0;

// convert wheel speed to local velocity
	math.MatrixMult((float*) qTOv, (float*) wheelspeed, 3, 4, 1,
			(float*) vlocal);

// convert local velocity to global velocity
	float c = cos(state[2]);
	float s = sin(state[2]);
	vglobal[0] = c * vlocal[0] - s * vlocal[1]; //matrix multiplication was wrong!!
	vglobal[1] = s * vlocal[0] + c * vlocal[1]; //
	vglobal[2] = vlocal[2];
	//Serial.print("V Global ");
	//Serial.println(vglobal[0], 6);
	//Serial.println(vglobal[1], 6);
	//Serial.println(vglobal[2], 6);

// update state
	dX = ((vglobal[0] + state[3]) / 2) * deltaT / 1000; //RK2 integration method
	dY = ((vglobal[1] + state[4]) / 2) * deltaT / 1000; //RK2 integration method
	dth = ((vglobal[2] + state[5]) / 2) * deltaT / 1000; //RK2 integration method
	//Serial.print("d-movement");
	//Serial.println(dX, 6);
	//Serial.println(dY, 6);
	//Serial.println(dth, 6);
	state[0] += dX;
	state[1] += dY;
	state[2] += dth;
	state[3] = vglobal[0];
	state[4] = vglobal[1];
	state[5] = vglobal[2];

	Serial.print("state: ");
	Serial.print(state[0], 6);
	Serial.print(" ");
	Serial.print(state[1], 6);
	Serial.print(" ");
	Serial.print(state[2], 6);
	Serial.print(" ");
	Serial.print(state[3], 6);
	Serial.print(" ");
	Serial.print(state[4], 6);
	Serial.print(" ");
	Serial.println(state[5], 6);
}

/**
 * Produces 10 setpoints given the 3 waypoints in the current segment
 */
void BezierInterp() {
	float seqTH[11];
	float orient[10];
	math.MatrixMult((float*) BCoeff, (float*) waypointX, 11, 4, 1,
			(float*) setpoint0);
	math.MatrixMult((float*) BCoeff, (float*) waypointY, 11, 4, 1,
			(float*) setpoint1);
	float timestep[11] = { 0.0 };
	float dx1, dy1, dx2, dy2, dxy1, dxy2, dir;
	dx1 = cos(state[2]);
	dy1 = sin(state[2]);
	dx2 = setpoint0[1] - setpoint0[0];
	dy2 = setpoint1[1] - setpoint1[0];
	dxy2 = sqrt(sq(dx2) + sq(dy2));
	dir = dx1 * dy2 - dx2 * dy1;
	if (dir >= 0) {
		seqTH[0] = acos((dx1 * dx2 + dy1 * dy2) / (dxy2));
	} else {
		seqTH[0] = -acos((dx1 * dx2 + dy1 * dy2) / (dxy2));
	}
	for (uint8_t i = 1; i <= 9; i = i + 1) {
		dx2 = setpoint0[i + 1] - setpoint0[i];
		dy2 = setpoint1[i + 1] - setpoint1[i];
		dxy2 = sqrt(sq(dx2) + sq(dy2));
		dx1 = setpoint0[i] - setpoint0[i - 1];
		dy1 = setpoint1[i] - setpoint1[i - 1];
		dxy1 = sqrt(sq(dx1) + sq(dy1));
		dir = dx1 * dy2 - dx2 * dy1;
		if (dir >= 0) {
			seqTH[i] = acos((dx1 * dx2 + dy1 * dy2) / (dxy1 * dxy2));
		} else {
			seqTH[i] = -acos((dx1 * dx2 + dy1 * dy2) / (dxy1 * dxy2));
		}
		timestep[i] = dxy1 / Vref;
	}
	timestep[10] = dxy2 / Vref;
	orient[0] = state[2] + seqTH[0];
	for (uint8_t i = 1; i <= 9; i = i + 1) {
		orient[i] = orient[i - 1] + seqTH[i];
	}
	for (uint8_t i = 1; i <= 9; i = i + 1) {
		setpoint2[i] = (orient[i] + orient[i - 1]) / 2;
		setpoint3[i] = Vref * cos(orient[i - 1]); //try Vref*cos(orient[count-1])cos(step2[count])
		setpoint4[i] = Vref * sin(orient[i - 1]);
	}
	setpoint2[10] = orient[9];
	setpoint3[10] = 0;
	setpoint4[10] = 0;
	setpoint5[10] = 0;
}

/**
 * set 10 setpoints at current location but with increasing angle
 */
void CircularObservation() {
	state[2] = atan2(sin(state[2]), cos(state[2])); //zero the angular position.
	setpoint0[0] = state[0];
	setpoint1[0] = state[1];
	setpoint2[0] = state[2];
	setpoint3[0] = 0;
	setpoint4[0] = 0;
	const float d = (float) 2 * PI / 10;
	for (uint8_t i = 1; i <= 10; i++) {
		setpoint0[i] = state[0];
		setpoint1[i] = state[1];
		setpoint2[i] = state[2] + d * i;
		setpoint3[i] = 0;
		setpoint4[i] = 0;
	}
}

void RecoveryMode() {
	float dx1;
	float dy1;
	float dx2;
	float dy2;
	float dxy1;
	float dxy2;
	float dir;
	dx1 = cos(state[2]);
	dy1 = sin(state[2]);
	dx2 = setpoint0[curr_sp] - state[0];
	dy2 = setpoint1[curr_sp] - state[1];
	dxy2 = sqrt(sq(dx2) + sq(dy2));
	dir = dx1 * dy2 - dx2 * dy1;
	if (dir >= 0) {
		setpoint2[curr_sp] = state[2] + acos((dx1 * dx2 + dy1 * dy2) / (dxy2));
	} else {
		setpoint2[curr_sp] = state[2] - acos((dx1 * dx2 + dy1 * dy2) / (dxy2));
	}
	setpoint3[curr_sp] = 0.1 * cos(setpoint2[curr_sp]);
	setpoint4[curr_sp] = 0.1 * sin(setpoint3[curr_sp]);
}

