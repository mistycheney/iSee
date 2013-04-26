// Do not remove the include below
#include "MasterMega2.h"

#include "message.h"

#define HOKUYO_DATA_LENGTH 2200 // 496 seems to be the maximum dynamic array size we can claim given the wifi library
char* hokuyo_data;

master_data md;
slave_data sd;

float waypointListX[6] = { 0.5, 1.0, 1.0, 1.0, 0.5, 0.0 };
float waypointListY[6] = { 0.5, 0.5, 0.0, -0.5, -0.5, 0.0 };
int n_waypoint = 6;

void setup() {

	Serial.begin(115200);

//	Notify(PSTR("initializing WiFi\n"));
//	wifi_init();
//
//	Notify(PSTR("waiting for connection\n"));
//	wifi_connect();
//
//	Notify(PSTR("setting up USB\n"));
//	usb_setup();
//
//	hokuyo_data = (char*) malloc(HOKUYO_DATA_LENGTH);
//	if (hokuyo_data) {
//		Notify(PSTR("hokuyo_data allocated\n"));
//	}
//
//	hokuyo_on();
//	Notify(PSTR("hokuyo on\n"));

	I2Csetup();
	Serial.println("master I2C set up");

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
//	usb_loop();
//	Notify(PSTR("start"));
//	if (!done) {
//		for (uint8_t i = 0; i < 1; i++) {
//	hokuyo_capture_send();

//		}
//	}
//	Notify(PSTR("complete"));
//	wifi_send();
//	delay(100);
}
