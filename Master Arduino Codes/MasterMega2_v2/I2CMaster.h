/*
 * I2C.h
 *
 *  Created on: May 18, 2012
 *      Author: yuncong
 */

#ifndef I2CMASTER_H_
#define I2CMASTER_H_

#include <Wire.h>
#include "datatypes.h"

void I2Csetup();
void I2Creceive(int numbytes);
void I2Csend();

void receive_float(float* f);

void print_struct(slave_data* sd);
void print_struct(master_data* md);

#endif /* I2CMASTER_H_ */
