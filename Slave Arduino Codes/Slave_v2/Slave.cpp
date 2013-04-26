#include "Slave.h"

master_data md;
slave_data sd;

void setup() {
	Serial.begin(115200);
	motor_setup();
	I2Csetup();

	md.x = 0;
	md.y = 0;
	md.theta = 0;
	md.status = 0;
	sd.waypointX[0] = 0;
	sd.waypointX[1] = 0;
	sd.waypointX[2] = 0;
	sd.waypointY[0] = 0;
	sd.waypointY[1] = 0;
	sd.waypointY[2] = 0;
	sd.status = 0;
}

void loop() {
	Serial.println("To start motor, press any key...");
	while (!Serial.available()) {
	}
	motor_go();
}

