
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "led.h"
#include <Arduino.h>

#include "pubsub.h"
#include "config.h"
#include <DW1000.h>



int ledPin = 13;
static void ledTask(void* args);

void ledInit()
{
    xTaskCreate(ledTask, "LT", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

void ledFail()
{
     pinMode(ledPin, OUTPUT);
     digitalWrite(ledPin, HIGH);
}

void ledTask(void* args) {
  portTickType lastWakeTime = xTaskGetTickCount();
   pinMode(ledPin, OUTPUT);

  while(true){
     digitalWrite(ledPin, LOW);
    vTaskDelayUntil(&lastWakeTime, M2T(500));
     digitalWrite(ledPin, HIGH);
    vTaskDelayUntil(&lastWakeTime, M2T(500));
    uint16_t errorCode = getErrorCode();
    if (errorCode != 0)
    {
      Serial.print(getId());
      Serial.print(" ");
    }
    if (errorCode & 1)      
      Serial.println("PubSub not loaded correctely");
    if (errorCode & (1 << 1))      
      Serial.println("MPU not present or faulty");
    if (errorCode & (1 << 2))      
      Serial.println("Attitude task not loaded correctely");
    if (errorCode & (1 << 3))      
      Serial.println("FlowDeck not present or faulty");
    if (errorCode & (1 << 4))      {
      Serial.println("DWM not present or faulty");
      DW1000.isValid();
    }
    if (errorCode & (1 << 5))      
      Serial.println("State Task not loaded correctely");
    if (errorCode & (1 << 6))      
      Serial.println("LDM task not loaded correctely");
    if (errorCode & (1 << 7))      
      Serial.println("NRF not present or faulty");
    if (errorCode & (1 << 8))      
      Serial.println("UART Task not loaded correctely");
    if (errorCode & (1 << 9))      
      Serial.println("Path Planning Task not loaded correctely");
    if (errorCode & (1 << 10))      
      Serial.println("Robot Task not loaded correctely");
  }
  
}

