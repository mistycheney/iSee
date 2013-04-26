#include "Slave.h"

master_data md;
slave_data sd;

void setup() {
	Serial.begin(115200);
	motor_setup();
	I2Csetup();

	// initialize data
	memset(&md, 0, sizeof(md));
	memset(&sd, 0, sizeof(sd));
	md.status = sd.status = FINSEG_MASK;

	Serial.println("To start motor, press any key...");
	while (Serial.available() == 0) {
	}
}

void loop() {
	motor_go();
}

