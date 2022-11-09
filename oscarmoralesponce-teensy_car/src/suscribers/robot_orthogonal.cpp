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


#define NORMAL_SPEED 75
#define NORMAL_LATERAL_SPEED 80

typedef struct Motor_T{
  uint8_t in1;
  uint8_t in2;
  uint8_t pwm;
} Motor;

static arm_pid_instance_f32 pidErrorX;

static Motor motorpwm[4] = {
		    {.in1 = _pinMotor1IN1,  .in2 = _pinMotor1IN2, .pwm = 0 },
		    {.in1 = _pinMotor2IN1,  .in2 = _pinMotor2IN2, .pwm = 0 }, 
		    {.in1 = _pinMotor3IN2,  .in2 = _pinMotor3IN1, .pwm = 0 }, 
		    {.in1 = _pinMotor4IN2,  .in2 = _pinMotor4IN1, .pwm = 0 }};




		    
void orthoCalibrate()
{
  
}


void orthoInit()
{
    uint8_t i;
    for (i=0; i<4 ; i++)
    {
      pinMode(motorpwm[i].in1, OUTPUT);
      pinMode(motorpwm[i].in2, OUTPUT);
      digitalWrite(motorpwm[i].in1, LOW);
      digitalWrite(motorpwm[i].in2, LOW);
    }
/*
    pidSpeed->Kp = 1.0f;
    pidSpeed->Ki = 0.0f;
    pidSpeed->Kd = 0.01f;

    
    pidCorrection->Kp = 10.0f; 
    pidCorrection->Ki = 0.00f;
    pidCorrection->Kd = 0.0f;
*/
    /*
     pidErrorX.Kp = 1.725f;
     pidErrorX.Ki = 0.0000225f;
     pidErrorX.Kd = 3.8f;*/
     
     
     pidErrorX.Kp = 5.0f;
     pidErrorX.Ki = 0.0f;
     pidErrorX.Kd = 0.0f;


      arm_pid_init_f32(&pidErrorX, 1);
}


void orthoDir(uint8_t dir, Motor *motor)
{
  
    
    switch (dir)
    {
      case BRAKE:
	
	  analogWrite(motor->in1, 0);
	  analogWrite(motor->in2, 0);
	  
	  digitalWrite(motor->in1, HIGH);
	  digitalWrite(motor->in2, HIGH);
 	  
	  break;
      case FORWARD:
	  analogWrite(motor->in1, motor->pwm);
	  analogWrite(motor->in2, 0);
	  digitalWrite(motor->in1, HIGH);
	  digitalWrite(motor->in2, LOW);
	  break;
      case BACKWARD:
	  analogWrite(motor->in1, 0);
	  analogWrite(motor->in2, motor->pwm);
	  digitalWrite(motor->in1, LOW);
	  digitalWrite(motor->in2, HIGH);
	  
	  break;
       case STANDBY:
	  analogWrite(motor->in1, 0);
	  analogWrite(motor->in2, 0);
	  digitalWrite(motor->in1, LOW);
	  digitalWrite(motor->in2, LOW);
	  break;
    }
}


void orthoRun(uint8_t dir, uint8_t )
{
    switch (dir)
    {
      case BRAKE:

	  orthoDir(BRAKE, &motorpwm[0]);
 	  orthoDir(BRAKE, &motorpwm[1]);
 	  orthoDir(BRAKE, &motorpwm[2]);
 	  orthoDir(BRAKE, &motorpwm[3]);
	  arm_pid_reset_f32(&pidErrorX);	  

	  break;
      case FORWARD:
	  
  	  orthoDir(FORWARD, &motorpwm[0]);
       	  orthoDir(FORWARD, &motorpwm[1]);
          orthoDir(FORWARD, &motorpwm[2]);
     	  orthoDir(FORWARD, &motorpwm[3]);
	  arm_pid_reset_f32(&pidErrorX);
	  break;
      case BACKWARD:
	  
 	  orthoDir(BACKWARD, &motorpwm[0]);
	  orthoDir(BACKWARD, &motorpwm[1]);
 	  orthoDir(BACKWARD, &motorpwm[2]);
	  orthoDir(BACKWARD, &motorpwm[3]);
	  arm_pid_reset_f32(&pidErrorX);
	  break;  
     case LEFT:
	 	  
 	  orthoDir(BACKWARD, &motorpwm[2]);
 	  orthoDir(FORWARD, &motorpwm[1]);
	  orthoDir(FORWARD, &motorpwm[3]);
	  orthoDir(BACKWARD, &motorpwm[0]);
	  arm_pid_reset_f32(&pidErrorX);
	  break;
      case RIGHT:
 	  orthoDir(BACKWARD, &motorpwm[3]);
 	  orthoDir(FORWARD, &motorpwm[0]);
  	  orthoDir(BACKWARD, &motorpwm[1]);
          orthoDir(FORWARD, &motorpwm[2]);
	  arm_pid_reset_f32(&pidErrorX);
	  break; 
      case ROTATE:
	arm_pid_reset_f32(&pidErrorX);
// 	  Serial.println(" ROTATE");


	  break;
      case STANDBY:
// 	  Serial.println(" STANDBY");
	  orthoDir(STANDBY, &motorpwm[0]);
	  orthoDir(STANDBY, &motorpwm[1]);
	  orthoDir(STANDBY, &motorpwm[2]);
	  orthoDir(STANDBY, &motorpwm[3]);
	  arm_pid_reset_f32(&pidErrorX);
	  break; 
    }

}



void orthoSetActuator(float speed, float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY, LDMap *ldmap)
{
    float motorX = 0.666;
    float motorY = 0.666;
    int i =0;
    int16_t pwmCorrecion[4];
	
    
    switch (dir)
    {
      case ROTATE:
     {
	int16_t pwmCorrecion = 80; //+ abs(correction);
        
// 	if (pwmCorrecion > 90)
// 	  pwmCorrecion = 90;
	
	
	for (i=0; i<4; i++)
	{
	  motorpwm[i].pwm = (uint8_t)pwmCorrecion;
	}
	
//  	Serial.print("Corre ");
// 	Serial.println(correction);  
		    
	
	if (correction > 0) // Rotate right	  
	{
	  orthoDir(FORWARD, &motorpwm[0]);
	  orthoDir(FORWARD, &motorpwm[3]);
	  orthoDir(BACKWARD, &motorpwm[1]);
 	  orthoDir(BACKWARD, &motorpwm[2]);
	}
	else
	{ 
	  
	  orthoDir(BACKWARD, &motorpwm[0]);
	  orthoDir(BACKWARD, &motorpwm[3]);
	  orthoDir(FORWARD, &motorpwm[1]);
 	  orthoDir(FORWARD, &motorpwm[2]);
	}

      }	
      break;
        case FORWARD:
	    
	    pwmCorrecion[0] = NORMAL_SPEED + (int8_t)speed + correction;
	    pwmCorrecion[1] = NORMAL_SPEED + (int8_t)speed - correction;
	    pwmCorrecion[2] = NORMAL_SPEED + (int8_t)speed - correction;
	    pwmCorrecion[3] = NORMAL_SPEED + (int8_t)speed + correction;  
	case BACKWARD:
	{
	    if (dir == BACKWARD)
	    {
	      pwmCorrecion[0] = NORMAL_SPEED + (int8_t)speed - correction;
	      pwmCorrecion[1] = NORMAL_SPEED + (int8_t)speed + correction;
	      pwmCorrecion[2] = NORMAL_SPEED + (int8_t)speed + correction;
	      pwmCorrecion[3] = NORMAL_SPEED + (int8_t)speed - correction;
	    }
	    
	    for (i=0; i<4; i++)
	    {
	      if (pwmCorrecion[i] < 1)
		pwmCorrecion[i] = 1;
	      if (pwmCorrecion[i] > 200)
		pwmCorrecion[i] = 2000;
	      motorpwm[i].pwm = (uint8_t)pwmCorrecion[i];
	    }
	    
	    orthoRun(dir, 0);
	}
        break;
	
	case RIGHT:
	 {
	    int16_t pwmCorrecion[4];
	    int8_t pwmDir[4];
	    
//  	    float correction2 = arm_pid_f32(&pidErrorX, *diffX * -1);
        //Serial.print(correction2);
        //Serial.print(" correction & diffX: ");
        //Serial.println(*diffX);
// 	    Serial.print("Correction: ");
// 	    Serial.println(correction);
		
	    pwmCorrecion[0] = NORMAL_LATERAL_SPEED + (int8_t)speed + correction; //- correction2; 
	    pwmCorrecion[1] = NORMAL_LATERAL_SPEED + (int8_t)speed + correction; //+ correction2;
	    pwmCorrecion[2] = NORMAL_LATERAL_SPEED + (int8_t)speed - correction;  //- correction2;
	    pwmCorrecion[3] = NORMAL_LATERAL_SPEED + (int8_t)speed - correction;  // + correction2; 
	
	    for (i=0; i<4; i++)
	    {
		pwmDir[i] = 1;
		if (pwmCorrecion[i] < 60)
		{
		    motorpwm[i].pwm = NORMAL_LATERAL_SPEED + max(pwmCorrecion[0] - 60, 0);
		    pwmDir[i] = 0;
		}
		else
		{
		  if (pwmCorrecion[i] > 200)
		    pwmCorrecion[i] = 200;
		  motorpwm[i].pwm = pwmCorrecion[i];
		}
	    }
	    if (pwmDir[0]) orthoDir(BACKWARD, &motorpwm[0]);
	    else orthoDir(FORWARD, &motorpwm[0]);
	    if (pwmDir[3]) orthoDir(FORWARD, &motorpwm[3]);
	    else orthoDir(BACKWARD, &motorpwm[3]);
	    
	    if (pwmDir[1]) orthoDir(FORWARD, &motorpwm[1]);
	    else orthoDir(BACKWARD, &motorpwm[1]);
	    if (pwmDir[2]) orthoDir(BACKWARD, &motorpwm[2]);
	    else orthoDir(FORWARD, &motorpwm[2]);
	  }

	  break;	

	case LEFT:
	 {
	    int16_t pwmCorrecion[4];
	    int8_t pwmDir[4];
	    
//  	    float correction2 = arm_pid_f32(&pidErrorX, *diffX * -1);
        //Serial.print(correction2);
// 	    Serial.print("Correction: ");
// 	    Serial.println(correction);
        //Serial.println(*diffX);
		
	    pwmCorrecion[0] = NORMAL_LATERAL_SPEED + (int8_t)speed - correction; //+ correction2; 
	    pwmCorrecion[1] = NORMAL_LATERAL_SPEED + (int8_t)speed - correction;  //- correction2;
	    pwmCorrecion[2] = NORMAL_LATERAL_SPEED + (int8_t)speed + correction;  //+ correction2;
	    pwmCorrecion[3] = NORMAL_LATERAL_SPEED + (int8_t)speed + correction;  // - correction2; 
	
	    for (i=0; i<4; i++)
	    {
		pwmDir[i] = 1;
		if (pwmCorrecion[i] < 70)
		{
		    motorpwm[i].pwm = NORMAL_LATERAL_SPEED + max(pwmCorrecion[0] - 70, 0);
		    pwmDir[i] = 0;
		}
		else
		{
		  if (pwmCorrecion[i] > 200)
		    pwmCorrecion[i] = 200;
		  motorpwm[i].pwm = pwmCorrecion[i];
		}
	    }
	    if (pwmDir[0]) orthoDir(FORWARD, &motorpwm[0]);
	    else orthoDir(BACKWARD, &motorpwm[0]);
	    if (pwmDir[3]) orthoDir(BACKWARD, &motorpwm[3]);
	    else orthoDir(FORWARD, &motorpwm[3]);
	    
	    if (pwmDir[1]) orthoDir(BACKWARD, &motorpwm[1]);
	    else orthoDir(FORWARD, &motorpwm[1]);
	    if (pwmDir[2]) orthoDir(FORWARD, &motorpwm[2]);
	    else orthoDir(BACKWARD, &motorpwm[2]);
	  }

	  break;	

	default:
	  for (i=0; i<4; i++)
	  {
	    motorpwm[i].pwm = 0;
	  }
	  orthoRun(dir, 0);
	  break;
     }
    
    
    float averageSpeed = (motorpwm[0].pwm + motorpwm[1].pwm + motorpwm[2].pwm + motorpwm[3].pwm) / 4.0f;
    
    *motorSpeedX = *motorSpeedY =  0.00;
    switch (dir)
    {
      case BRAKE:
	  *motorSpeedX = 0;
	  break;
      case FORWARD:
      case BACKWARD:
	  *motorSpeedX = averageSpeed * motorX;
	  break;  
      case LEFT:
      case RIGHT:
	  *motorSpeedY = averageSpeed * motorY;
	  break; 
      case STANDBY:
	  *motorSpeedX = 0;
	  break; 
    }
 
}

