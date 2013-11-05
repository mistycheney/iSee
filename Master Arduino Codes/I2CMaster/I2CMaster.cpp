// Do not remove the include below
#include "I2CMaster.h"

const uint8_t I2C_SLAVE_ADDRESS = 9;
const uint8_t I2C_MASTER_ADDRESS = 6;

slave_data sd;
master_data md;

void setup() {
	Serial.begin(115200);

	Wire.begin(I2C_MASTER_ADDRESS);
	Wire.onRequest(I2Csend);
	Wire.onReceive(I2Creceive);

//	Serial.println("type something to start; you should get the Uno ready");
//	while (Serial.available() == 0) {
//	};
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
	*f = 1;
//	uint32_t q = 0;
//	for (uint8_t i = 0; i < 4; i++) {
//		byte c = Wire.read();
//		q |= ((uint32_t) c << 8 * i);
//	}
//	*f = reinterpret_cast<float&>(q);
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

void I2Csend() {
	Serial.println("send ");

	sd.status = md.status;
	if (md.status & FINSEG_MASK) {
		sd.waypointX[0] = 1;
		sd.waypointX[1] = 1;
		sd.waypointX[2] = 1;
		sd.waypointY[0] = 1;
		sd.waypointY[1] = 1;
		sd.waypointY[2] = 1;
		sd.status |= NEWDATA_MASK;
	}

	Serial.println("send ");
	print_struct(&sd);

	uint8_t size = sizeof(slave_data);
	byte* b = (byte*) malloc(size);
	memcpy(b, &sd, size);
	uint8_t size_sent = Wire.write(b, size);
	free(b);

	Serial.print(size_sent);
	Serial.println(" bytes");

	delay(100);
}
