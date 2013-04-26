// Do not remove the include below
#include "MasterMega2.h"

#include "WiFiShield.h"
#include "USBShield.h"
//#include "I2C.h"
#include <Wire.h>

#include "message.h"

#define HOKUYO_DATA_LENGTH 2200 // 496 seems to be the maximum dynamic array size we can claim given the wifi library
char* hokuyo_data;

#define I2C_SLAVE_ADDRESS 9
#define I2C_MASTER_ADDRESS 6

void setup() {

	Serial.begin(115200);

	Notify(PSTR("initializing WiFi\n"));
	wifi_init();

	Notify(PSTR("waiting for connection\n"));
	wifi_connect();

	Notify(PSTR("setting up USB\n"));
	usb_setup();

	hokuyo_data = (char*) malloc(HOKUYO_DATA_LENGTH);
	if (hokuyo_data) {
		Notify(PSTR("hokuyo_data allocated\n"));
	}

	hokuyo_on();
	Notify(PSTR("hokuyo on\n"));

	Wire.begin(I2C_MASTER_ADDRESS);
	Wire.onReceive(receive);
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

void receive(int numBytes) {
	Serial.print("received ");

	uint32_t q = 0;
	while (Wire.available()) {
		byte c = Wire.read();
		q = (q << 8) | c;
	}
	Serial.println(reinterpret_cast<float&>(q));
}

