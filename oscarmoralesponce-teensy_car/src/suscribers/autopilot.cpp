/**
* Autopilot. 
*
* @author  Oscar Morales-Ponce
* @version 0.1
* @since   02-26-2019 
* 
* It registers as a subscriber to the NRF_DATA that it is triggered when a message arrives and the leaderId and active members every round of NRF
* It registers as a subscriber to the STATE_DATA that it is triggered when the robot computes a new state
* 
*/


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "config.h"

#include "robot.h"
#include "autopilot.h"
#include "virtualmachine.h"


#define SIZE_OF_QUEUE 3
#define MAXROUNDSINVOTING 3



// This goes in vm Mahine
extern uint8_t systemReady;
extern bool blocking;

static TaskHandle_t ldmHandle = NULL;
static QueueHandle_t stateQueue;
static QueueHandle_t rxQueue;

static void autopilotTask(void* args);
static void ldmTask(void* args);


Control_Data controlData;
LDMap ldmap; 

uint8_t cooperativeState;
uint8_t operationMode;
uint8_t membersInProgress;
uint8_t membersInSync;
uint8_t membersInProgress2;
int8_t roundsInVoting;
uint32_t clock;
uint8_t propose;
uint8_t membersControl;
uint8_t failures;


uint32_t geClock() 
{
  return clock;
}

uint8_t  getOperationMode()
{
    return operationMode;
}
uint8_t getCooperativeState()
{
    return cooperativeState;
}

void setOperationMode(uint8_t os)
{
    operationMode = os;
}

uint8_t getMembersInProgress()
{
    return membersInProgress;
}

uint8_t getMembersInSync()
{
    return membersInSync;
}

void autopilotInit()
{
    cooperativeState = IN_PROGRESS;
    operationMode = AUTONOMOUS;
    
    controlData.leaderId = getId();
    controlData.members = 1 << (getId() -1);
    controlData.state = 0;    
    controlData.time = 0;

    roundsInVoting = 0;
    membersInProgress = 0; 
    membersInSync = 0;
    clock = 0;
    
    blocking = false;

    xTaskCreate(autopilotTask, "autopilot", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(ldmTask, "LDM", configMINIMAL_STACK_SIZE, NULL, 3, &ldmHandle);
     
    stateQueue  = xQueueCreate(1, sizeof(LDMap));
    rxQueue = xQueueCreate(1, sizeof(LDMap));
     
    registerSubscriber(NRF_DATA, ldmHandle, rxQueue);
    registerSubscriber(STATE_DATA, ldmHandle, stateQueue);
}

bool autopilotTest()
{
      return true;
}

void setInstruction(UARTCommand *command)
{
     RobotInstructionState instructionS;
     instructionS.maxMilliseconds = command->maxTime;
     instructionS.currentInstruction = command->dir;
     instructionS.speed = command->speed;
     instructionS.distance = command->distance;
     instructionS.orientation = command->orientation;
     instructionS.stop = command->stop;
     instructionS.stopCondition = command->stopCondition;
     instructionS.adaptingOrientation = command->adaptingOrientation;

     robotSetInstruction(&instructionS);	     
}

void cooperativeTIOA()
{

    switch (cooperativeState)
    {
      case IN_PROGRESS:
        if (robotGetStatus()) 
        {
      #if NRF_TASK == 1
            membersInSync |= 1 << (getId() -1);
            stopSending();
            cooperativeState = SYNC;
            failures = 0;
       	    //Serial.println("InProgress ");
      #else	  
            blocking = false;
            newRound();
      #endif
	  }
	  break;
	  
      case SYNC:
      if (membersInSync == membersInProgress)
      {
          
          cooperativeState = VOTING;
          roundsInVoting = MAXROUNDSINVOTING;
          membersInProgress = 0;
          controlData.members = 0;
          membersControl = 0;
          //operationMode = COOPERATIVE;
            //Serial.print("SYNC ");
          //Serial.print(membersInSync);
          //Serial.print(" , " );
          //Serial.print(membersInProgress);
            //Serial.print(" , " );
          //Serial.println(getOperationMode());
      }
      if (controlData.members != membersControl)
      {
          failures++;
          if (failures > 2)
          {
              cooperativeState = VOTING;
              roundsInVoting = MAXROUNDSINVOTING;
              membersInProgress = 0;
              controlData.members = 0;
              membersControl = 0;
              operationMode = AUTONOMOUS;
              //Serial.println("Autonomous");
          }
      }
      break;
          case VOTING:
              if (roundsInVoting <= 0)
              {	
                  //Serial.print("Progess ");
                  //Serial.println(membersInSync);
                  cooperativeState = IN_PROGRESS;
                  membersInProgress |= 1 << (getId() -1);
                  membersInSync = 0;
                  newRound();
                  uint8_t j=0, k=0, i;
                  for (i=1; i<= LDM_SIZE; i++)
                  {
                      LDMap *ldmap = getLDMP(i);
                      if (ldmap->id > 0)
                      {   
                          k++;
                          if (ldmap->operationMode != operationMode)
                              break;
                          else
                              j++;
                      }
                  }
                  if (j == k)
                  {
                      operationMode = COOPERATIVE; 
                  }
                  else 
                  {
                      operationMode = AUTONOMOUS; 
                  }
                  startSending();
                  blocking = false;
                  
                  
                  //operationMode = COOPERATIVE;
      
              }
              roundsInVoting--;
              break;
      }
}

void autopilotTask(void* args)
{		
	portTickType lastWakeTime = xTaskGetTickCount();

	while (!systemReady)
	{
	  vTaskDelayUntil(&lastWakeTime, M2T(10));
	}
    

	membersInProgress = 1 << (getId() -1);	    
	newRound();
	startSending();
  while (true) 
	{
 	    clock++;
	    vTaskDelayUntil(&lastWakeTime, M2T(128));
      //

// 	    membersInProgress = 1 << (getId() -1);
	    cooperativeTIOA();    
        
      }
}



void ldmTask(void* args)
{
    uint32_t publisherId;
    LDMap ldmapState;
    
    
    while (true)
    {
	if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
	{
	  switch (publisherId)
	  {
	    case NRF_DATA:
	      if (xQueueReceive(rxQueue, &ldmapState, ( TickType_t ) 0 ) == pdTRUE)
	      {		
		  if (ldmapState.id == getId())
		  {
                controlData.members = ldmapState.members;
                controlData.leaderId = ldmapState.leaderId;
                if (cooperativeState == IN_PROGRESS)
                  membersControl |= controlData.members;

		  }
		  else
		  {
		      //lastTime = ldmapState.clock;    
		      ldmapState.id = ldmapState.id -1;
		      switch (cooperativeState)
		      {
                case IN_PROGRESS:
                  switch (ldmapState.tioaState)
                  {
                      case IN_PROGRESS:
                        membersInProgress |= (1 << ldmapState.id);
                        if (ldmapState.operationMode == AUTONOMOUS)
                        {
                            operationMode = AUTONOMOUS;
//                             Serial.println("AUTONOMOUS");
                        }
                        break;
                          case SYNC:
                            membersInProgress |= (1 << ldmapState.id);
        // 				if (ldmapState.members == membersInProgress)
        // 				  membersInSync |= (1 << ldmapState.id);
                            if (ldmapState.operationMode == AUTONOMOUS)
                                operationMode = AUTONOMOUS;
                          
                        break;
                          case VOTING:
                        break;
                      }		
                      break;
                    case SYNC:
                      switch (ldmapState.tioaState)
                      {
                          case IN_PROGRESS:
                        membersInProgress |= (1 << ldmapState.id);
                        break;
                          case SYNC:  
                        membersInProgress |= (1 << ldmapState.id);
                        if (ldmapState.members == membersInProgress)
                            membersInSync |= (1 << ldmapState.id);

                        break;
                          case VOTING:	

                        cooperativeState = VOTING;
                        roundsInVoting = MAXROUNDSINVOTING -1;
                        membersInProgress = 0;
                        controlData.members = 0;
                        membersControl = 0;
                        operationMode = COOPERATIVE;
//                         Serial.println("Progress");

                        break;
                      }
                      break;
                    case VOTING:		
                      switch (ldmapState.tioaState)
                      {
                          case IN_PROGRESS:
        // 				membersInProgress |= (1 << ldmapState.id);
                        break;
                          case SYNC:
                          case VOTING:	
                        break;
                      }
			     break;
		      }   
		      insertLDMAP(0, &ldmapState);
		  }	
	      }
	      break;
	    case STATE_DATA:
 	      if (xQueueReceive(stateQueue, &ldmap, ( TickType_t ) 0 ) == pdTRUE)
	      {
             if (ldmap.id > 0)
             {
                ldmap.id = ldmap.id -1;
                memcpy(&ldmap.messageData, getMessageData(), sizeof(MessageData));
                insertLDMAP(0, &ldmap);
             }
	      }
	  
	      break;
	  }
	}
   }
}
