#ifndef   __STATE_TASK__
#define   __STATE_TASK__

void stateInit();
bool stateTest();

void setMotorSpeed(float motorX, float motorY, float angle);
void setMotorReset();

void ekf_reset();

#endif
