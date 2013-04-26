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

const uint8_t ENDPATH_BIT = 1;
const uint8_t FINSEG_BIT = 2;
const uint8_t CIRCOBS_BIT = 4;
const uint8_t NEWDATA_BIT = 8;

extern master_data md;
extern slave_data sd;

extern float waypointListX[6];
extern float waypointListY[6];
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
		q = (q << 8) | c;
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
	sd.status = md.status;
	if (md.status & FINSEG_BIT) {
		if (curr_wp + 3 < n_waypoint) {
			sd.waypointX[0] = waypointListX[curr_wp];
			sd.waypointY[0] = waypointListY[curr_wp];
			sd.waypointX[1] = waypointListX[curr_wp + 1];
			sd.waypointY[1] = waypointListY[curr_wp + 1];
			sd.waypointX[2] = waypointListX[curr_wp + 2];
			sd.waypointY[2] = waypointListY[curr_wp + 2];
			sd.status |= NEWDATA_BIT;
			curr_wp += 3;
		} else {
			Serial.print("Not enough waypoints");
			while (!Serial.available()) {
			}
		}
	}

	Serial.println("send");
	print_struct(&sd);

	uint8_t size = sizeof(slave_data);
	byte* b = (byte*) malloc(size);
	memcpy(b, &sd, size);
	Wire.write(b, size);
	free(b);

	delay(100);
}
