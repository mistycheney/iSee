// Do not remove the include below
#include "I2CSlave.h"

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9
#define I2C_MASTER_ADDRESS 6

slave_data sd;
master_data md;

void setup() {
	Serial.begin(115200);

	Wire.begin();

	Serial.println("type something to start; you should get the Mega ready");
	while (Serial.available() == 0) {
	};

	I2Csend();
//	I2Creceive();
}

void loop() {
	delay(100);
}

void print_struct(slave_data* a) {
	Serial.print("slave data: ");
}

void print_struct(master_data* a) {
	Serial.print("master data: ");
}

void receive_float(float* f) {
	uint32_t q = 0;
	for (uint8_t i = 0; i < 4; i++) {
		byte c = Wire.read();
		q |= ((uint32_t) c << 8 * i);
	}
	*f = reinterpret_cast<float&>(q);
}

void I2Creceive() {
	int numBytes = Wire.requestFrom(I2C_MASTER_ADDRESS, sizeof(slave_data));
	Serial.print("received ");
	Serial.print(numBytes);
	Serial.println(" bytes");

	if (Wire.available()) {
		for (uint8_t j = 0; j < 3; j++) {
			receive_float(&(sd.waypointX[j]));
		}
		for (uint8_t j = 0; j < 3; j++) {
			receive_float(&(sd.waypointY[j]));
		}
		sd.status = Wire.read();
	}
	print_struct(&sd);
}

void I2Csend() {
	Serial.print("send ");
	print_struct(&md);

	Wire.beginTransmission(I2C_MASTER_ADDRESS); // each begin/endTransmission pair invokes the receive callback once.

	uint8_t size = sizeof(master_data);
	byte* b = (byte*) malloc(size);
	memcpy(b, &md, size);
	uint8_t size_sent = Wire.write(b, size);
	free(b);

	Serial.print(size_sent);
	Serial.println(" bytes");

	uint8_t tx_status = Wire.endTransmission();
	Serial.print("tx status: ");
	Serial.println(tx_status);
	delay(100);
}
