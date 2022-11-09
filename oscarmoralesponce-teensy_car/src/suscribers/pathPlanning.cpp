#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"


static bool bInit = false;
static void pathPlanningTask(void* args);
QueueHandle_t xLocalCommand;
TaskHandle_t pathPlanningHandler;
uint8_t cp = 0;

UARTCommand nullCommand = { .maxTime= 1000,  .speed = 0, .orientation = 0, .distance=0, .dir = STANDBY, .stop = false, .temp1=0, .temp2=0, .stopCondition = NULL};



void pathPlanningInit()
{
    if (bInit == false)
    {
        xTaskCreate(pathPlanningTask, "pathPlanning", configMINIMAL_STACK_SIZE, NULL, 3, &pathPlanningHandler);
         xLocalCommand = xQueueCreate(1, sizeof(UARTCommand));
        //xQueueSend( xLocalCommand, ( void * ) &nullCommand, ( TickType_t ) 0 );
      
          registerSubscriber(UART_COMMAND, pathPlanningHandler, xLocalCommand);
      
       bInit = true;
    }
}

bool pathPlanningTest()
{
    
    return bInit;
}

bool getNullCommand(UARTCommand *command)
{
  
    memcpy(command, &nullCommand, sizeof(UARTCommand));
     return true;
    
}

bool getCommand(UARTCommand *command)
{
    UARTCommand comm;
    if (xQueueReceive( xLocalCommand, &comm, ( TickType_t ) 0 ) == pdTRUE)
    {
        //Serial.print("Comman");
        //Serial.print(comm.speed);
        memcpy(command, &comm, sizeof(UARTCommand));
        return true;
    }
    return false;
}

void pathPlanningTask(void* args)
{
    uint32_t publisherId;

    while (true) {
        if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
        {
            
        }
    }
}
