#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <Arduino.h>
#include "pubsub.h"
#include "datastructures.h"


#define trigPin 12
#define echoPin 11

static bool bInit = false;
static void distanceTask(void* args);

Distance_Data obstacle_data;

// Distance_Data *getDistanceData() { return &distance_data; }



void distanceInit()
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    xTaskCreate(distanceTask, "DIST", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    bInit = registerPublisher(OBSTACLE_DATA, 0, NULL);
}

bool distanceTest()
{
    return bInit;
}


void distanceTask(void* args) 
{
    uint32_t lastWakeTime = xTaskGetTickCount();
    float timetaken;
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, 60/portTICK_RATE_MS);
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        timetaken = (float)pulseIn(echoPin, HIGH);
        
        obstacle_data.distance = (timetaken/2.0) * 29.1;
    //     Serial.print("Distance:");
    //     Serial.print(timetaken);
    //     Serial.print("\n");
        if (obstacle_data.distance > 0 && obstacle_data.distance < 200)
            publish(OBSTACLE_DATA);
    }
}
