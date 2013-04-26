#ifndef DATATYPES_H_
#define DATATYPES_H_

typedef struct { // data used by slave
	float waypointX[3];
	float waypointY[3];
	byte status;
} slave_data;

typedef struct { //data used by master
	float x;
	float y;
	float theta;
	byte status;
} master_data;

#endif