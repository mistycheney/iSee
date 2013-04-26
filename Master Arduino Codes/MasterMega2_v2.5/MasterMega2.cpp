// Do not remove the include below
#include "MasterMega2.h"

#include "message.h"

#define HOKUYO_DATA_LENGTH 2200 // 496 seems to be the maximum dynamic array size we can claim given the wifi library
char* hokuyo_data;

master_data md;
slave_data sd;

int n_waypoint = 6;
// remember to free these two arrays
float* waypointListX = (float*) malloc(n_waypoint * sizeof(float));
float* waypointListY = (float*) malloc(n_waypoint * sizeof(float));

void setup() {

	float wp_x[] = { 0.5, 1.0, 1.0, 1.0, 0.5, 0.0 };
	float wp_y[] = { 0.5, 0.5, 0.0, -0.5, -0.5, 0.0 };
	memcpy(waypointListX, wp_x, n_waypoint * sizeof(float));
	memcpy(waypointListY, wp_y, n_waypoint * sizeof(float));

	Serial.begin(115200);

	Notify(PSTR("setting up USB\n"));
	usb_setup();

	hokuyo_data = (char*) malloc(HOKUYO_DATA_LENGTH);
	if (hokuyo_data) {
		Notify(PSTR("hokuyo_data allocated\n"));
	}

	hokuyo_on();
	Notify(PSTR("hokuyo on\n"));

	I2Csetup();
	Notify(PSTR("Master I2C set up\n"));

	// initialize data
	memset(&md, 0, sizeof(md));
	memset(&sd, 0, sizeof(sd));
	md.status = sd.status = FINSEG_MASK;

	Notify(PSTR("initializing WiFi\n"));
	wifi_init();

	Notify(PSTR("waiting for connection\n"));
	wifi_connect();

}

void loop() {
//	usb_loop();
//	Notify(PSTR("start"));
//	if (!done) {
//		for (uint8_t i = 0; i < 1; i++) {
	hokuyo_capture_send();

//		}
//	}
//	Notify(PSTR("complete"));
//	wifi_send();
//	delay(100);
}
