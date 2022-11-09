/* This example shows how to use continuous mode to take
  range measurements with the VL53L0X. It is based on
  vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

  The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

#define NUM_OF_SENSORS 8

VL53L0X sensor[NUM_OF_SENSORS];
static byte initialPins = 2;
uint16_t distance[NUM_OF_SENSORS];
static byte initialAddress = 0x30;

byte numSensors = 8;

void setup()
{
  int i;
  
  Serial.begin(115200);
  Wire.begin();

  for (i=0; i<NUM_OF_SENSORS; i++)
  {
    pinMode(initialPins + i, OUTPUT);
    digitalWrite(initialPins + i, HIGH);
    delay(10);
    if (sensor[i].readReg16Bit(0xC0) == 61098)
    {
    
        sensor[i].setAddress(initialAddress+i);
    }
    else
    {
        numSensors = i;
        break;    
    }
  }

  for (i=0; i<numSensors; i++)
  {
     sensor[i].init(true);
     sensor[i].setTimeout(500);
     sensor[i].startContinuous();
  }
}
   

void loop()
{
  int i;
   for (i=0; i<numSensors; i++)
   {
      distance[i] =  sensor[i].readRangeContinuousMillimeters()/10;
   }
   Serial.print("BE");
   for (i=0; i<numSensors; i++)
   {
        Serial.print(",");
       Serial.print(distance[i]);
   }
   Serial.println(",END");
   
}
