/*
 * I2C.cpp
 *
 *  Created on: May 18, 2012
 *      Author: yuncong
 */

#include "Arduino.h"
#include "I2CMaster.h"

const uint8_t I2C_SLAVE_ADDRESS = 9;
const uint8_t I2C_MASTER_ADDRESS = 6;

extern master_data md;
extern slave_data sd;

extern float* waypointListX;
extern float* waypointListY;
extern int n_waypoint;

int curr_wp = 0;

//extern void wifi_send(char* str, uint16_t len);
//extern void wifi_get(char* str, uint16_t len);

void I2Csetup() {
	Wire.begin(I2C_MASTER_ADDRESS);
	Wire.onReceive(I2Creceive);
	Wire.onRequest(I2Csend);
}

void receive_float(float* f) {
	uint32_t q = 0;
	for (uint8_t i = 0; i < 4; i++) {
		byte c = Wire.read();
		q |= ((uint32_t) c << 8 * i);
	}
	*f = reinterpret_cast<float&>(q);
}

void print_struct(slave_data* a) {
	Serial.print("slave data: ");
	for (uint8_t i = 0; i < 3; i++) {
		Serial.print(a->waypointX[i]);
		Serial.print(" ");
		Serial.print(a->waypointY[i]);
		Serial.print(" ");
	}
	Serial.println(a->status, 2);
}

void print_struct(master_data* a) {
	Serial.print("master data: ");
	Serial.print(a->x);
	Serial.print(" ");
	Serial.print(a->y);
	Serial.print(" ");
	Serial.print(a->theta);
	Serial.print(" ");
	Serial.println(a->status, 2);
}

/**
 * Receive master data
 */
void I2Creceive(int numBytes) {
	Serial.print("received ");
	Serial.print(numBytes);
	Serial.println(" bytes");

	while (Wire.available()) {
		receive_float(&(md.x));
		receive_float(&(md.y));
		receive_float(&(md.theta));
		md.status = Wire.read();
	}
	print_struct(&md);
}

/**
 * Send slave data (x,y of 3 waypoints + 1 status byte)
 */
void I2Csend() {
	if (curr_wp + 2 < n_waypoint) {
		memcpy(sd.waypointX, &(waypointListX[curr_wp]), 3 * sizeof(float));
		memcpy(sd.waypointY, &(waypointListY[curr_wp]), 3 * sizeof(float));
		curr_wp += 3;
	} else {
		memset(sd.waypointX, 0, n_waypoint * sizeof(float));
		memset(sd.waypointY, 0, n_waypoint * sizeof(float));
		set(ENDPATH_MASK);
	}

	sd.status = md.status;

	Serial.println("send ");
	print_struct(&sd);

	uint8_t size = sizeof(slave_data);
	byte* b = (byte*) malloc(size);
	memcpy(b, &sd, size);
	long sent_size = Wire.write(b, size); // if read the return value of write, hangs the program.... Weird...
	free(b);

	delay(100);
}
