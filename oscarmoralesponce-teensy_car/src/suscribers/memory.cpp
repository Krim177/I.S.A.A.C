#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <arm_math.h>
#include <EEPROM.h>
#include "datastructures.h"
#include "pubsub.h"
#include "config.h"

TaskHandle_t memoryHandle;
static QueueHandle_t memoryQueue;
static bool bInit = false;
static void memoryTask(void* args);

void memoryInit()
{
    if (bInit == false)
    {
	xTaskCreate(memoryTask, "MEMORY", configMINIMAL_STACK_SIZE, NULL, 3, &memoryHandle);
	bInit = true;
	memoryQueue = xQueueCreate(1, sizeof(UARTMemory));
	registerSubscriber(UART_MEMORY, memoryHandle, memoryQueue);
    }
}

bool memoryTest()
{
    return bInit;
}


void memoryTask(void* args)
{
    uint32_t publisherId;
    UARTMemory memory;
    uint8_t i;
    

    while (true)
    {
        if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
        {

	    if (xQueueReceive(memoryQueue, &memory, ( TickType_t ) 0 )  == pdTRUE)
	    {
  		    //Serial.print(memory.address );
  		    //Serial.print(":");
 		    //Serial.println(memory.buffer[0]);
	    	
	    	for (i=0; i<memory.length; i++)
    		{
 	    	    Serial.println(memory.buffer[i]);
		        EEPROM.write(memory.address + i, memory.buffer[i]);
		    }
            raedParameter();
            setPIdParamenters();;
// 		Serial.print("\n");
		    
	    }
	}   
    }
}
