/*
 * I2C.h
 *
 *  Created on: May 18, 2012
 *      Author: yuncong
 */

#ifndef I2CSLAVE_H_
#define I2CSLAVE_H_

#include <Wire.h>
#include "datatypes.h"

void I2Csetup();
void I2Creceive();
void I2Csend();

void receive_float(float* f);

void print_struct(slave_data* sd);
void print_struct(master_data* md);


#endif /* I2CSLAVE_H_ */
