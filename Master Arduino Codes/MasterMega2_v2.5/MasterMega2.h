// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef SocketTest_H_
#define SocketTest_H_
#include "Arduino.h"

//add your includes for the project SocketTest here

#include "WiFiShield.h"
#include "USBShield.h"
#include "I2CMaster.h"

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

extern master_data md;
extern slave_data sd;

extern float* waypointListX;
extern float* waypointListY;
extern int n_waypoint;


//add your function definitions for the project SocketTest here

//Do not add code below this line
#endif /* SocketTest_H_ */
