//
//  Aerospace
//  

#ifndef _AEROSPACE_H_
#define _AEROSPACE_H_


#define CHECK 1

#define LENGTH 25  //CM width of the car
// #define INFINITE 819  
#define FAULTY 9999
#define DISTANCE_THERSHOLD 300

#define P 0.85f
#define DISTANCE_OVERPASS_LEFT 12
#define DISTANCE_OVERPASS_RIGHT 12
#define DISTANCE_CORRECTION 200   // MOVE_LEFT_UNITL_FREE, CORRECTION_STATE
#define MAX_OBJECT_SIZE 300   // MOVE_LEFT_UNITL_FREE, HORIZONTAL
#define MAX_ROBOT_DISTANCE 300   
#define MAX_OBJECT_DISTANCE 200
#define ALIGN_THRESHOLD 5

#define MIN_DISTANCE_HORIZONTAL 0
#define MAX_DISTANCE_HORIZONTAL  80
#define MAX_DISTANCE_X  300

#define MIN_DISTANCE_PUSHING 0
#define MAX_DISTANCE_PUSHING 20
#define UPPER_BOUND_DISTANCE 150
#define MAX_DIFFERENCE 20
#define TEST_DISTANCE 20

#define ALIGN_PROPORTIONAL 0.01f    //  0.346f
#define ALIGN_INTEGRAL 0.0f
#define ALIGN_DERIVATE 0.0f

#define DISTANCE_INHERTIA 10

#define sqrt2  1.4142135


#define areBothSensorsInBounds() (externalSensor.dist_0 <= DISTANCE_THERSHOLD && externalSensor.dist_1 <= DISTANCE_THERSHOLD && \
  abs(externalSensor.dist_0 - externalSensor.dist_1) <= MAX_DIFFERENCE)
#define isAtLeastOneSensorsInBound() (externalSensor.dist_0 <= DISTANCE_THERSHOLD || externalSensor.dist_1 <= DISTANCE_THERSHOLD)
#define isAtLeastOneSensorsOutOfBound() (externalSensor.dist_0 > DISTANCE_THERSHOLD || externalSensor.dist_1 > DISTANCE_THERSHOLD)

#define areBothSensorsOutOfBounds() (externalSensor.dist_0 > DISTANCE_THERSHOLD && externalSensor.dist_1 > DISTANCE_THERSHOLD)


enum { INIT=1, END, SCANNING, BROADCAST, APPROACHING, PREPARE_TO_MOVE, RELOCATE_ON_FACE,  CHANGE_FACE, PUSH_BOX, 
       TEST_PUSH_BOX, WAIT, WAIT_TEST, PUSHING, INTERPUSH, STABILIZE, OBSTACLE_AVOIDANCE, READY_TO_PUSH, DEBUG, PATH_FOLLOWING};

enum { HORIZONTAL_LEFT = 100, HORIZONTAL_RIGHT, HORIZONTAL_CORRECTION, HORIZONTAL, VERTICAL, ROTATE_CAR, CORRECTION_STATE, ENTER_FACE, CENTER, WAITING, MOVE_LEFT_UNITL_FREE, MOVE_RIGHT_UNITL_FREE, RESET_STATE, ALIGN, SCANNINGROTATE, CANNOT_MOVE, F_CORRECTION};

enum {BEGINING_TRAVERSE, FULLY_TRAVERSE, OVERPASS_THE_BOX };


      
typedef struct _StateData {
 	int8_t  offset;
	uint8_t rank;
	uint8_t direction;
	uint8_t active;
	int16_t angle;
	int16_t distanceToMove;
	int16_t distanceBetweenRobots;
	uint8_t boxLength;
	uint8_t boxWidth;
	int8_t currentFace;
	uint8_t previousRankFailures;
	int16_t Xdistance;
	int16_t Ydistance;
	bool    isRobotInCorridor;
	uint8_t members;
	uint8_t localState;
	uint8_t stateSecondLevel;  
	int16_t initialDistance;
	int8_t faceTraverse;
	int8_t errorMovements;
	bool XDone, YDone; 
} StateData;

void debugPrintState(uint8_t state);
void debugPrint(uint8_t globalState, LDMap *ldmap);
void debugMode(uint8_t *globalState);

#endif /* _AEROSPACE_H_ */
