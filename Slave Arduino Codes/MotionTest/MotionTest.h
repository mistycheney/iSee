// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef MotionTest_H_
#define MotionTest_H_
#include "Arduino.h"
//add your includes for the project MotionTest here

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

//add your function definitions for the project MotionTest here


extern master_data md;
extern slave_data sd;


//Do not add code below this line
#endif /* MotionTest_H_ */
