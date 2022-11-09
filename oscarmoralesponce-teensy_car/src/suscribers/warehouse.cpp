#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "config.h"
#include "state.h"
#include "robot.h"
#include "uartCommand.h"
#include <arm_math.h>
#include "Aerospace.h"

#if APPLICATION  == WAREHOUSE_ROBOT	

#define __DEBUG__
      
      
 
static SemaphoreHandle_t mutex;
ExternalSensor externalSensor;
StateData stateData;



/*********************************************************************************************
*  OnExternalSensorOrtho receives the reading from the external computer, i.e., Raspberry PI
* Params: 
*     externalSensorReading: is the reading that the external comptuer sends
* 	globalState: The state of the main state machine
*******************************************************************************************/
uint8_t OnExternalSensorOrtho(ExternalSensor *externalSensorReading, LDMap *ldmap, uint8_t *globalState)
{
    xSemaphoreTake(mutex, portMAX_DELAY);



    xSemaphoreGive(mutex);
    return 0;
}


void onInitAerospace()
{
    mutex = xSemaphoreCreateMutex();
    
    

}

void sendMessages(uint8_t globalState)
{
    MessageData messageData;
    /*
    messageData.param1 = SET_PARAM1(stateData.rank, stateData.active);
    messageData.param2 = SET_PARAM2(externalSensor.code, stateData.currentFace, stateData.isRobotInCorridor);
    messageData.Xdistance = stateData.Xdistance;
    messageData.Ydistance = stateData.Ydistance;
    messageData.globalState = globalState;
    messageData.boxWidth = stateData.boxWidth;
    messageData.boxLength = stateData.boxLength;*/
    sendMessageData(&messageData);
}


// Main state machine
uint8_t onNewOrthoRobot(CarCommand *command, LDMap *ldmap, uint8_t globalState, uint32_t clock)
{
   
    //xSemaphoreTake(mutex, portMAX_DELAY);
    //sendTaskToExternalComputer(externalSensor.code, globalState, stateData.localState);
    
    //xSemaphoreGive(mutex);
    return 0;
}



// Main state machine
uint8_t onEndOrthoRobot(LDMap *ldmap, uint8_t *globalState, uint32_t clock, uint8_t errCode, int16_t lastDistance, int16_t lastAngle)
{	 

    //xSemaphoreGive(mutex);
    
    return 0;
}

#endif