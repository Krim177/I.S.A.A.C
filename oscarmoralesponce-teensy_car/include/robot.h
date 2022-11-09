#ifndef   __ROBOT_TASK__
#define   __ROBOT_TASK__

#include "autopilot.h"

void robotInit();
bool robotTest();

void resetFailureCounter();
uint8_t getFailures();
void robotSetInstruction(RobotInstructionState *ins);
uint8_t robotGetStatus();



void roverCalibrate();
void roverInit();
void roverRun(uint8_t dir, uint8_t );
void roverSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY,  LDMap *ldmap);


void orthoCalibrate();
void orthoInit();
void orthoRun(uint8_t dir, uint8_t );
void orthoSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY,  LDMap *ldmap);

void orthoSuspensionCalibrate();
void orthoSuspensionInit();
void orthoSuspensionRun(uint8_t dir, uint8_t );
void orthoSetSuspensionActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY,  LDMap *ldmap);

void carCalibration();
void carInit();
void carRun(uint8_t dir, uint8_t prevDir);
void carSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY,  LDMap *ldmap);


#endif
