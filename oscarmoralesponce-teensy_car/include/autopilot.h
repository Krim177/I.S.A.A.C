#ifndef _AUTOPILOT_H_
#define _AUTOPILOT_H_

#include <FreeRTOS.h>
#include <task.h>


uint32_t geClock();
uint8_t  getOperationMode();
uint8_t getCooperativeState();
uint8_t getMembersInProgress();
uint8_t getMembersInSync();
void setOperationMode(uint8_t os);


typedef struct RobotInstructionState_T
{
	uint32_t maxMilliseconds;
	uint32_t totalMilliseconds;
	float distance;
	float speed;
	float orientation;
	uint8_t currentInstruction;
	bool stop;
	
	bool (*stopCondition)();
	 void (*adaptingOrientation)(Quaternion *p, Quaternion *q);
	
} RobotInstructionState;


void autopilotInit();
bool autopilotTest();
void setInstruction(UARTCommand *command);


#endif // ROBOT_H_INCLUDED
