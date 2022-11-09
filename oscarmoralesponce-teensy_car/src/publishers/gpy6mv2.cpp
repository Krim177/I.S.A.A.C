#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <TinyGPS.h>
#include <Arduino.h>


#include "gps.h"
#include "pubsub.h"
#include "datastructures.h"


TinyGPS gps;
GPS_Data gps_data;

static bool bInit = false;
static void gpsTask(void* args);


void gps6mv2Init()
{  
    Serial2.begin(9600);
    xTaskCreate(gpsTask, "GPST", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    bInit = true;
}

bool gps6mv2Test()
{
    return bInit;
}


void gpsTask(void* args) 
{
  uint32_t lastWakeTime = xTaskGetTickCount();
  bool newdata = false;
  while (true) {
    vTaskDelayUntil(&lastWakeTime, 1/portTICK_RATE_MS);
    if (Serial2.available())
    {
      if (gps.encode(Serial2.read()))
   	newdata = true;
    }

    if (newdata) {
      gps.f_get_position(&gps_data.latitude, &gps_data.longitude, &gps_data.time);
      publish(GPS_DATA);
      
//       DEBUG("%f,%f", flat, flon);
      newdata = false;
    }
  }
}
