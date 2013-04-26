// Do not remove the include below
#include "MotionTest.h"


master_data md;
slave_data sd;
//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(115200);
	motor_setup();

	// initialize data
	memset(&md, 0, sizeof(md));
	memset(&sd, 0, sizeof(sd));
	md.status = sd.status = FINSEG_MASK;

	Serial.println("To start motor");
	while (Serial.available() == 0) {
	}
}

// The loop function is called in an endless loop
void loop() {
	motor_go();
}
