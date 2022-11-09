#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <PWMServo.h>
#include <arm_math.h>

#include "config.h"



#define MAX_ANGLE 130
#define MIN_ANGLE 50
#define CENTER    100

#define MAX_SPEED 179
#define MIN_SPEED 1
#define NEUTRAL_SPEED 90


enum {BRAKE_STATE=1, NEUTRAL_STATE, FORWARD_STATE, BACKWARD_STATE};

int robotcarState = NEUTRAL_STATE;


void carCalibration()
{

}


void carInit()
{
      Serial3.begin(9600);
      robotcarState = NEUTRAL_STATE;
      
}

void carRun(uint8_t dir, uint8_t prevDir)
{
     robotcarState = dir;
     
    switch (dir)
    {
      case BRAKE_STATE:
      
        robotcarState = BRAKE_STATE;
	  break;
      case FORWARD_STATE:
	  robotcarState = FORWARD_STATE;
	  break;
      case BACKWARD:
	  robotcarState = BACKWARD;
	  break;
      case NEUTRAL_STATE:
	  robotcarState = NEUTRAL_STATE;
	  break;
      default:
	;
    } 
}

void carSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY, LDMap *ldmap)
{
      //float motorX = 0.001;    	
      int speedCar = NEUTRAL_SPEED;
      int steeringAngle = 0;
      
      switch (robotcarState)
      {
            case BRAKE_STATE:
                  if (abs(speed) < 3)
                        robotcarState = NEUTRAL_STATE;
                  else
                        speedCar = NEUTRAL_STATE;
	            break;
            case FORWARD_STATE:
                  speedCar = NEUTRAL_SPEED + speed;
                  steeringAngle = rotation;
                  break;
            case BACKWARD_STATE:
                  speedCar = NEUTRAL_SPEED - speed;
                  steeringAngle = rotation;
                        
                  break;
            case NEUTRAL_STATE:
                  

                  break;
            default:
                  ;
      }

      Serial.print("Speed ");   
      Serial.println(ldmap->speed);

      char message[5];
      message[0] = '#';
      message[1] = (char)speedCar;
      message[2] = (char)steeringAngle;
      message[3] = ';';
      Serial3.write(message[0]);
      Serial3.write(message[1]);
      Serial3.write(message[2]);
      Serial3.write(message[3]);
      Serial3.write('\0');
      /*Serial.print("Speed\t");
      Serial.print(speed);
      Serial.print("\t");
      Serial.println(ldmap->speed);
      */
}
    
