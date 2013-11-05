// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef I2CMaster_H_
#define I2CMaster_H_
#include "Arduino.h"
//add your includes for the project I2CMaster here

#include <Wire.h>
#include "datatypes.h"


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project I2CMaster here

void I2Creceive(int numbytes);
void I2Csend();

void receive_float(float* f);

void print_struct(slave_data* sd);
void print_struct(master_data* md);


//Do not add code below this line
#endif /* I2CMaster_H_ */
