/*
 * I2C.cpp
 *
 *  Created on: May 18, 2012
 *      Author: yuncong
 */

#include "Arduino.h"
#include "I2CSlave.h"

const uint8_t I2C_SLAVE_ADDRESS = 9;
const uint8_t I2C_MASTER_ADDRESS = 6;

extern master_data md;
extern slave_data sd;

void I2Csetup() {
	Wire.begin();
}

/**
 * Receive a float.
 * Note that AVR internal storage is little-endian, meaning MSB is stored in high address
 */
void receive_float(float* f) {
	uint32_t q = 0;
	for (uint8_t i = 0; i < 4; i++) {
		byte c = Wire.read();
		q |= ((uint32_t)c << 8 * i);
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
 * Receive slave data
 */
void I2Creceive() {
	int numBytes = Wire.requestFrom(I2C_MASTER_ADDRESS, sizeof(slave_data));
	Serial.print("received ");
	Serial.print(numBytes);
	Serial.println(" bytes");

	if (Wire.available()) {
		for (uint8_t j = 0; j < 3; j++) {
			receive_float(&(sd.waypointX[j]));
			receive_float(&(sd.waypointY[j]));
		}
		sd.status = Wire.read();
	}
	print_struct(&sd);
}

/**
 * Send master data
 */
void I2Csend() {
	md.status = sd.status;

	Serial.print("send ");
	print_struct(&md);

	Wire.beginTransmission(I2C_MASTER_ADDRESS); // each begin/endTransmission pair invokes the receive callback once.

	uint8_t size = sizeof(master_data);
	byte* b = (byte*) malloc(size);
	memcpy(b, &md, size);
	Wire.write(b, size);
	free(b);

	uint8_t tx_status = Wire.endTransmission();
	Serial.print("tx status: ");
	Serial.println(tx_status);
	delay(100);
}
