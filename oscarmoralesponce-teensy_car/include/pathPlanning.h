#ifndef   __PATHPLANNING_TASK__
#define   __PATHPLANNING_TASK__
#include "uartCommand.h"

void pathPlanningInit();
bool pathPlanningTest();
bool getCommand(UARTCommand *command);
bool getAutoCommand(UARTCommand *command);
bool getOrthoCommand(UARTCommand *command);
bool getRoverCommand(UARTCommand *command);
bool getNullCommand(UARTCommand *command);




#endif
