#ifndef   __CONFIG_TASK__
#define   __CONFIG_TASK__

#include "pubsub.h"
#include "datastructures.h"
#include "uartCommand.h"


// publihers
#include "distance.h"
#include "gps.h"
#include "mpu.h"
#include "dwm.h"
#include "flow.h"

// subscribers
#include "robot_car.h"
#include "pathPlanning.h"
#include "uartCommand.h"
#include "nrf51822.h"
#include "dwm.h"
#include "ldm.h"
#include "attitude.h"
#include "report.h"       
#include "state.h"

#define LDM_SIZE 6

#define ORTHOCAR 2
#define ORTHOSUSPENSIONCAR 5
#define CAR 1
#define ROVERCAR 3


#define UART_TASK  1	
#define NRF_TASK  0
#define DWM_TASK   0		
#define FLOW_TASK  0


#define INTERNAL_CONTROLLER 1
#define EXTERNAL_CONTROLLER 2
#define HYBRID_CONTROLLER 3
#define CONTROLLER EXTERNAL_CONTROLLER


#define EXTERNAL_SENSOR 1
#define INTERNAL_SENSOR 2
#define SENSORS EXTERNAL_SENSOR



#define AEROSPACE 1
#define WAREHOUSE_ROBOT 2
#define AUTONOMOUS_CAR 3
#define APPLICATION AUTONOMOUS_CAR

void raedParameter();
void setPIdParamenters();
#endif
