// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef Slave_H_
#define Slave_H_
#include "Arduino.h"

#include "I2CSlave.h"
#include "motor.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project Slave here

extern master_data md;
extern slave_data sd;

//Do not add code below this line
#endif /* Slave_H_ */
