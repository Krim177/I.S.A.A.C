//
//  L293D.c
//  
//
//  Created by Oscar Morales Ponce on 11/21/17.
//
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <Arduino.h>
#include <arm_math.h>


#include "config.h"

#define _pinMotor1IN1  3
#define _pinMotor1IN2  4
#define _pinMotor2IN1  5
#define _pinMotor2IN2  6
#define _pinMotor3IN1  20
#define _pinMotor3IN2  21
#define _pinMotor4IN1  22
#define _pinMotor4IN2  23

#define STOP_CAR 0

typedef struct Motor_T{
  uint8_t in1;
  uint8_t in2;
  uint8_t pwm;
} Motor;


Motor motorroverpwm[4] = {
		    {.in1 = _pinMotor1IN1,  .in2 = _pinMotor1IN2, .pwm = 0 },
		    {.in1 = _pinMotor2IN1,  .in2 = _pinMotor2IN2, .pwm = 0 }, 
		    {.in1 = _pinMotor3IN1,  .in2 = _pinMotor3IN2, .pwm = 0 }, 
		    {.in1 = _pinMotor4IN1,  .in2 = _pinMotor4IN2, .pwm = 0 }};
		    
void roverCalibrate()
{
  
}

void roverInit()
{
    uint8_t i;
    for (i=0; i<4 ; i++)
    {
      pinMode(motorroverpwm[i].in1, OUTPUT);
      pinMode(motorroverpwm[i].in2, OUTPUT);
      digitalWrite(motorroverpwm[i].in1, LOW);
      digitalWrite(motorroverpwm[i].in2, LOW);
    }
    
//     pidSpeed->Kp = 0.7f;
//     pidSpeed->Ki = 0.05f; //0.0027f;
//     pidSpeed->Kd = 0.00f; //0.05f;
//     
//     pidCorrection->Kp = 15.0f; 
//     pidCorrection->Ki = 0.1f;
//     pidCorrection->Kd = -0.5f;
}

void roverDir(uint8_t dir, Motor *motor)
{
    switch (dir)
    {
      case BRAKE:
	  analogWrite(motor->in1, 0);
	  analogWrite(motor->in2, 0);
	  digitalWrite(motor->in1, LOW);
	  digitalWrite(motor->in2, LOW);
	  break;
      case FORWARD:
	  analogWrite(motor->in1, motor->pwm);
	  digitalWrite(motor->in2, LOW);
	  break;
      case BACKWARD:
	  digitalWrite(motor->in1, LOW);
	  analogWrite(motor->in2, motor->pwm);
	  break;
       case STANDBY:
	  digitalWrite(motor->in1, LOW);
	  digitalWrite(motor->in2, LOW);
	  break;
    }
}


void roverRun(uint8_t dir, uint8_t)
{
    switch (dir)
    {
      case BRAKE:
	  roverDir(BRAKE, &motorroverpwm[0]);
	  roverDir(BRAKE, &motorroverpwm[1]);
	  roverDir(BRAKE, &motorroverpwm[2]);
	  roverDir(BRAKE, &motorroverpwm[3]);
	  break;
      case FORWARD:
	  roverDir(FORWARD, &motorroverpwm[0]);
	  roverDir(FORWARD, &motorroverpwm[1]);
	  roverDir(FORWARD, &motorroverpwm[2]);
	  roverDir(FORWARD, &motorroverpwm[3]);
	  break;
      case BACKWARD:
	  roverDir(BACKWARD, &motorroverpwm[0]);
	  roverDir(BACKWARD, &motorroverpwm[1]);
	  roverDir(BACKWARD, &motorroverpwm[2]);
	  roverDir(BACKWARD, &motorroverpwm[3]);
	  break;  
     case LEFT:
	  roverDir(BACKWARD, &motorroverpwm[0]);
	  roverDir(FORWARD,   &motorroverpwm[3]);
	  roverDir(BACKWARD, &motorroverpwm[2]);
	  roverDir(FORWARD,   &motorroverpwm[1]);
	  
	  break;
      case RIGHT:
	  roverDir(FORWARD,   &motorroverpwm[2]);
 	  roverDir(BACKWARD, &motorroverpwm[1]);
	  roverDir(FORWARD,   &motorroverpwm[0]);
 	  roverDir(BACKWARD, &motorroverpwm[3]);
	  break; 
      case STANDBY:
	  roverDir(STANDBY, &motorroverpwm[0]);
	  roverDir(STANDBY, &motorroverpwm[1]);
	  roverDir(STANDBY, &motorroverpwm[2]);
	  roverDir(STANDBY, &motorroverpwm[3]);
	  break; 
    }
}

void roverSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY, LDMap *ldmap)
{
  
    float motorX = -1.08;
    float motorY = 1;
    float pwmCorrecionl,  pwmCorrecionr;

    pwmCorrecionl = speed + correction;
    pwmCorrecionr = speed - correction;

    if (pwmCorrecionl < 0)
      pwmCorrecionl = 0;
    if (pwmCorrecionr < 0)
      pwmCorrecionr = 0;
    if (pwmCorrecionl > 255)
      pwmCorrecionl = 255;
    if (pwmCorrecionr > 255)
      pwmCorrecionr = 255;
    
    motorroverpwm[0].pwm = (uint8_t)pwmCorrecionr;
    motorroverpwm[3].pwm = (uint8_t)pwmCorrecionr;
    motorroverpwm[1].pwm = (uint8_t)pwmCorrecionl;
    motorroverpwm[2].pwm = (uint8_t)pwmCorrecionl;
    
    *motorSpeedX = *motorSpeedY  = 0.00;
    if (dir == BACKWARD)
	*motorSpeedX = -pwmCorrecionl * motorX; 
    else if (dir == FORWARD)
	*motorSpeedX = pwmCorrecionl * motorX;	
    else if (dir == RIGHT)
 	*motorSpeedX = motorY * acc;
    else if (dir == LEFT )
 	*motorSpeedX = -motorY * acc; 
}

  