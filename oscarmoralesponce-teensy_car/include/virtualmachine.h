#ifndef _VIRTUALMACHINE_H_
#define _VIRTUALMACHINE_H_

#include <FreeRTOS.h>
#include <task.h>

#include "datastructures.h"


void virtualMachineInit();

MessageData *getMessageData();


#endif // ROBOT_H_INCLUDED
