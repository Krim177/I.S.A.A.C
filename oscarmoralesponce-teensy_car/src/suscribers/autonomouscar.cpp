#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "config.h"
#include "state.h"
#include "robot.h"
#include "uartCommand.h"
#include <arm_math.h>
#include "autonomouscar.h"
 
#if APPLICATION  == AUTONOMOUS_CAR	 

#define __DEBUG__
      
enum {  START_ALGO= 50, NEXT_STEP}; 
      
int timerRounds = 2;
static SemaphoreHandle_t mutex;
//ExternalSensor externalSensor;
//StateData stateData;



/*********************************************************************************************
*  OnExternalSensorOrtho receives the reading from the external computer, i.e., Raspberry PI
* Params: 
*     externalSensorReading: is the reading that the external comptuer sends
* 	globalState: The state of the main state machine
*******************************************************************************************/
uint8_t OnExternalSensorCar(ExternalSensor *externalSensorReading, LDMap *ldmap, uint8_t *globalState)
{
    //xSemaphoreTake(mutex, portMAX_DELAY);



    //xSemaphoreGive(mutex);
    return 0;
}


void onInitCar()
{
    mutex = xSemaphoreCreateMutex();
    
    

}

void sendMessages(uint8_t globalState)
{
    //MessageData messageData;
    /*
    messageData.param1 = SET_PARAM1(stateData.rank, stateData.active);
    messageData.param2 = SET_PARAM2(externalSensor.code, stateData.currentFace, stateData.isRobotInCorridor);
    messageData.Xdistance = stateData.Xdistance;
    messageData.Ydistance = stateData.Ydistance;
    messageData.globalState = globalState;
    messageData.boxWidth = stateData.boxWidth;
    messageData.boxLength = stateData.boxLength;*/
    //endMessageData(&messageData);
}


// Main state machine
uint8_t onNewCarRobot(CarCommand *command, LDMap *ldmap, uint8_t globalState, uint32_t clock)
{
   
    xSemaphoreTake(mutex, portMAX_DELAY);
    //sendTaskToExternalComputer(externalSensor.code, globalState, stateData.localState);
    switch (globalState)
    {
        case INIT:
            //command->dir = BRAKE;
            //command->stop = true;
            break;
        case CALIBRATION:
            carCalibration();
            break;
        case START_ALGO: 
            //command->dir = FORWARD; 
            break;
        case NEXT_STEP:
            command->dir = BRAKE; 
            command->stop = true;
            break;
    }
    xSemaphoreGive(mutex); 
    return 0;
}



// Main state machine
uint8_t onEndCarRobot(LDMap *ldmap, uint8_t *globalState, uint32_t clock, uint8_t errCode, int16_t lastDistance, int16_t lastAngle)
{	 

    xSemaphoreTake(mutex, portMAX_DELAY);
    switch (*globalState)
    {

      case INIT:

            if (timerRounds > 0)
                timerRounds--;
            else 
            {
                *globalState = START_ALGO;
                 timerRounds = 10;
            }
          break;

        case CALIBRATION:
            *globalState = START_ALGO;
            break;
        

        case START_ALGO: 
            if (timerRounds < 0)
                timerRounds--;
           {
                *globalState = NEXT_STEP;
                 timerRounds = 5000;
            }
          break;

          
        case NEXT_STEP: 
            if (timerRounds < 0)
                timerRounds--;
           {
                *globalState = START_ALGO;
                 timerRounds = 200;
            }
            break;
    
    }
    xSemaphoreGive(mutex); 
    return 0;
}

#endif