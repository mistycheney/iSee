#ifndef DATATYPES_H_
#define DATATYPES_H_

//void printPROGMEM(char const* msg);

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

extern master_data md;
extern slave_data sd;

const uint8_t ENDPATH_MASK = 1; // whether the robot reaches the endpoint
const uint8_t FINSEG_MASK = 2; // whether the last waypoint in a segment is reached
const uint8_t CIRCOBS_MASK = 4; // whether the robot is doing circular observation
const uint8_t NEWDATA_MASK = 8; // means the robot has a set of new waypoints in the buffer, but haven't used it

inline boolean isSet(byte status, byte mask) {
	return status & mask;
}

inline boolean isNEWDATA() {
	return isSet(md.status, NEWDATA_MASK);
}
inline boolean isFINSEG() {
	return isSet(md.status, FINSEG_MASK);
}
inline boolean isENDPATH() {
	return isSet(md.status, ENDPATH_MASK);
}
inline boolean isCIRCOBS() {
	return isSet(md.status, CIRCOBS_MASK);
}

inline void set(const byte mask) {
	md.status |= mask;
	sd.status |= mask;
}

inline void clear(const byte mask) {
	md.status &= ~mask;
	sd.status &= ~mask;
}

#endif
