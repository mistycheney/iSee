#ifndef DATATYPES_H_
#define DATATYPES_H_

typedef struct { // data used by slave
	float waypointX[3];
	float waypointY[3];
	float slam_x;
	float slam_y;
	float slam_theta;
	byte status;
} slave_data;

typedef struct { //data used by master
	float odom_x;
	float odom_y;
	float odom_theta;
	byte status;
} master_data;

extern master_data md;
extern slave_data sd;

const uint8_t ENDPATH_MASK = 1; // whether the robot reaches the endpoint
const uint8_t FINSEG_MASK = 2; // whether the last waypoint in a segment is reached
const uint8_t CIRCOBS_MASK = 4; // whether the robot is doing circular observation
const uint8_t NEWDATA_MASK = 8; // means the robot has a set of new waypoints in the buffer, but haven't used it

inline boolean check(byte mask) {
	return sd.status & mask;
}

inline void set(byte mask) {
	md.status |= mask;
	sd.status |= mask;
}

inline void clear(byte mask) {
	md.status &= ~mask;
	sd.status &= ~mask;
}

#endif
