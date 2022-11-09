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


#define NORMAL_SPEED 25
#define NORMAL_LATERAL_SPEED 35
#define ROTATION_SPEED 65
#define THRESDHOLD 65

extern Parameters parameters;


typedef struct Motor_T{
  uint8_t in1;
  uint8_t in2;
  uint8_t pwm;
} Motor;

static Motor motorpwm[4] = {
		    {.in1 = _pinMotor1IN1,  .in2 = _pinMotor1IN2, .pwm = 0 },
		    {.in1 = _pinMotor2IN1,  .in2 = _pinMotor2IN2, .pwm = 0 }, 
		    {.in1 = _pinMotor3IN2,  .in2 = _pinMotor3IN1, .pwm = 0 }, 
		    {.in1 = _pinMotor4IN2,  .in2 = _pinMotor4IN1, .pwm = 0 }};

arm_pid_instance_f32 pidErrorX;
static float errorX;
static float correction2 = 0;

static void flowSensorTask(void* args);
static TaskHandle_t flowSenHandle = NULL;
static QueueHandle_t flowQueue;
static uint8_t currentDir;
		    
void orthoSuspensionCalibrate()
{
  
}


void orthoSuspensionInit()
{
   uint8_t i;
    for (i=0; i<4 ; i++)
    {
      pinMode(motorpwm[i].in1, OUTPUT);
      pinMode(motorpwm[i].in2, OUTPUT);
      digitalWrite(motorpwm[i].in1, LOW);
      digitalWrite(motorpwm[i].in2, LOW);
    }/*

    pidSpeed->Kp = 4.0f;
    pidSpeed->Ki = 0.0f;
    pidSpeed->Kd = 0.1f;

    
    pidRotation->Kp = 4.0; 
    pidRotation->Ki = 0.00001f;
    pidRotation->Kd = 3.0f;
    
    pidCorrection->Kp = 12.0f; 
    pidCorrection->Ki = 0.000001f;
    pidCorrection->Kd = 0.1f;*/
    
    pidErrorX.Kp = 0.06f;
    pidErrorX.Ki = 0.0;
    pidErrorX.Kd = 0.0001f;
    
    arm_pid_init_f32(&pidErrorX, 1);
    
     xTaskCreate(flowSensorTask, "FLOWSENS", configMINIMAL_STACK_SIZE, NULL, 3, &flowSenHandle);  
     flowQueue  = xQueueCreate(1, sizeof(Flow_Data));
     registerSubscriber(FLOW_DATA, flowSenHandle, flowQueue);
    
}


void flowSensorTask(void* args)
{
    uint32_t publisherId;
    while (true)	
    {
      if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
      { 
	Flow_Data flowData;
	
        if (xQueueReceive(flowQueue, &flowData, ( TickType_t ) 0 )  == pdTRUE)
        {
            if ((currentDir == LEFT || currentDir == RIGHT))
            {
                errorX += flowData.deltax; 
                correction2 = arm_pid_f32(&pidErrorX, errorX);
			/*
		Serial.print(flowData.deltax);
		Serial.print(", ");
		Serial.print(errorX);
		Serial.print(", ");
		Serial.println(correction2);*/
	
            }
            else
            {
                errorX = 0.0;
                correction2 = 0.0;
                arm_pid_reset_f32(&pidErrorX);
            }
        }
      }
    }
}


void orthoSuspensionDir(uint8_t dir, Motor *motor)
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


void orthoSuspensionRun(uint8_t dir, uint8_t )
{
    switch (dir)
    {
      case BRAKE:

	  orthoSuspensionDir(BRAKE, &motorpwm[0]);
 	  orthoSuspensionDir(BRAKE, &motorpwm[1]);
 	  orthoSuspensionDir(BRAKE, &motorpwm[2]);
 	  orthoSuspensionDir(BRAKE, &motorpwm[3]);
	  break;
      case FORWARD:
	  
  	  orthoSuspensionDir(FORWARD, &motorpwm[0]);
      	  orthoSuspensionDir(FORWARD, &motorpwm[1]);
       	  orthoSuspensionDir(FORWARD, &motorpwm[2]);
     	  orthoSuspensionDir(FORWARD, &motorpwm[3]);
	  break;
      case BACKWARD:
	  
 	  orthoSuspensionDir(BACKWARD, &motorpwm[0]);
	  orthoSuspensionDir(BACKWARD, &motorpwm[1]);
 	  orthoSuspensionDir(BACKWARD, &motorpwm[2]);
	  orthoSuspensionDir(BACKWARD, &motorpwm[3]);
	  break;  
     case LEFT:
	 	  
 	  orthoSuspensionDir(BACKWARD, &motorpwm[2]);
 	  orthoSuspensionDir(FORWARD, &motorpwm[1]);
	  orthoSuspensionDir(FORWARD, &motorpwm[3]);
	  orthoSuspensionDir(BACKWARD, &motorpwm[0]);
	  break;
      case RIGHT:
 	  orthoSuspensionDir(BACKWARD, &motorpwm[3]);
 	  orthoSuspensionDir(FORWARD, &motorpwm[0]);
  	  orthoSuspensionDir(BACKWARD, &motorpwm[1]);
          orthoSuspensionDir(FORWARD, &motorpwm[2]);
	  break; 
      case ROTATE:

	  break;
      case STANDBY:
	  orthoSuspensionDir(STANDBY, &motorpwm[0]);
	  orthoSuspensionDir(STANDBY, &motorpwm[1]);
	  orthoSuspensionDir(STANDBY, &motorpwm[2]);
	  orthoSuspensionDir(STANDBY, &motorpwm[3]);
	  break; 
    }

}



void orthoSetSuspensionActuator(float speed,  float lateralSpeed, float rotation, float correction, float acc, uint8_t dir, float *motorSpeedX, float *motorSpeedY, LDMap *ldmap)
{
    float motorX = 0.666;
    float motorY = 0.666;
    int i =0;
    int32_t pwmCorrecion[4];
	
    currentDir = dir; 
    switch (dir)
    {
      case BRAKE: 
	  orthoSuspensionDir(BRAKE, &motorpwm[0]);
	  orthoSuspensionDir(BRAKE, &motorpwm[1]);
 	  orthoSuspensionDir(BRAKE, &motorpwm[2]);
	  orthoSuspensionDir(BRAKE, &motorpwm[3]);
	  break;
      case ROTATE:
      {
	  int16_t pwmCorrecion = ROTATION_SPEED +  abs(rotation);
	  for (i=0; i<4; i++)
	  {
	    motorpwm[i].pwm = (uint8_t)pwmCorrecion;
	  }
	    
	  if (rotation > 3) // Rotate right	  
	  {
	    orthoSuspensionDir(FORWARD, &motorpwm[0]);
	    orthoSuspensionDir(FORWARD, &motorpwm[3]);
	    orthoSuspensionDir(BACKWARD, &motorpwm[1]);
	    orthoSuspensionDir(BACKWARD, &motorpwm[2]);
	  }
	  else if (rotation < -3) // Rotate right	  
	  {  
	    orthoSuspensionDir(BACKWARD, &motorpwm[0]);
	    orthoSuspensionDir(BACKWARD, &motorpwm[3]);
	    orthoSuspensionDir(FORWARD, &motorpwm[1]);
	    orthoSuspensionDir(FORWARD, &motorpwm[2]);
	  }
	  else 
	  {
	     orthoSuspensionDir(BRAKE, &motorpwm[0]);
	     orthoSuspensionDir(BRAKE, &motorpwm[1]);
	     orthoSuspensionDir(BRAKE, &motorpwm[2]);
	     orthoSuspensionDir(BRAKE, &motorpwm[3]);
	  }
	}	
      break;
	  case FORWARD:
	      
	      pwmCorrecion[0] = NORMAL_SPEED + parameters.offset[1] + (int8_t)speed + correction;
	      pwmCorrecion[1] = NORMAL_SPEED + parameters.offset[1] + (int8_t)speed - correction;
	      pwmCorrecion[2] = NORMAL_SPEED + parameters.offset[1] + (int8_t)speed - correction;
	      pwmCorrecion[3] = NORMAL_SPEED + parameters.offset[1] + (int8_t)speed + correction;  
	  
	      
	      for (i=0; i<4; i++)
	      {
		if (pwmCorrecion[i]  < NORMAL_SPEED)
		{
		  motorpwm[i].pwm = 0;
		  orthoSuspensionDir(BRAKE, &motorpwm[i]);
  // 		Serial.print("Breaking");
		}
		else 
		{
		  motorpwm[i].pwm = min((uint8_t)pwmCorrecion[i], 200);
		  orthoSuspensionDir(FORWARD, &motorpwm[i]);
		}
	      }
	      
	      break;
	      
	  case BACKWARD:
	  {
	      pwmCorrecion[0] = NORMAL_SPEED + parameters.offset[1] + (int16_t)speed; //- correction;
	      pwmCorrecion[1] = NORMAL_SPEED + parameters.offset[1] + (int16_t)speed; //+ correction;
	      pwmCorrecion[2] = NORMAL_SPEED + parameters.offset[1] + (int16_t)speed;// + correction;
	      pwmCorrecion[3] = NORMAL_SPEED + parameters.offset[1] + (int16_t)speed; // - correction;
	    
  //    	    Serial.print(pwmCorrecion[0]);
  //    	    Serial.print("PWM");
  //  	    Serial.println(speed);
	      for (i=0; i<4; i++)
	      {
		if (pwmCorrecion[i]  < NORMAL_SPEED)
		{
  // 		motorpwm[i].pwm = min(NORMAL_SPEED-(uint8_t)pwmCorrecion[i], 200);
		  motorpwm[i].pwm = 0;
		  orthoSuspensionDir(BRAKE, &motorpwm[i]);
  // 		orthoSuspensionDir(FORWARD, &motorpwm[i]);
  // 		Serial.print("Breaking");
		}
		else 
		{
		  motorpwm[i].pwm = min((uint8_t)pwmCorrecion[i], 200);
		  orthoSuspensionDir(BACKWARD, &motorpwm[i]);
		}
	      }
	  }
	  break;
	  
	  case RIGHT:
	  {
	      int16_t pwmCorrecion[4];
	      int8_t pwmDir[4];
	      
	  //motorOffset.offset[1] + 

	      pwmCorrecion[0] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed - correction - correction2; 
	      pwmCorrecion[1] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed - correction + correction2; 
	      pwmCorrecion[2] = NORMAL_LATERAL_SPEED + parameters.offset[0] +lateralSpeed + correction - correction2; 
	      pwmCorrecion[3] = NORMAL_LATERAL_SPEED + parameters.offset[0] +lateralSpeed + correction + correction2; 
		      
	      for (i=0; i<4; i++)
	      {
		pwmDir[i] = 1;
  // 	      if (pwmCorrecion[i] <= 10)
  // 		pwmCorrecion[i] = 10;
		if (pwmCorrecion[i] < NORMAL_LATERAL_SPEED)
		{
  //  		  pwmCorrecion[i] = NORMAL_LATERAL_SPEED + max(THRESDHOLD - pwmCorrecion[i], 0);
		    pwmDir[i] = 0;
		}
		if (pwmCorrecion[i] > 200)
		    pwmCorrecion[i] = 200;
		motorpwm[i].pwm = pwmCorrecion[i];
  // 	      Serial.print(pwmDir[0]);
  // 	      Serial.print(", ");  
  // 	      Serial.print(pwmCorrecion[0]);
  // 	      Serial.print(": ");  
	      }
  // 	    Serial.println();

	      if (pwmDir[0]) orthoSuspensionDir(BACKWARD, &motorpwm[0]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[0]);
	      if (pwmDir[1]) orthoSuspensionDir(FORWARD, &motorpwm[1]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[1]);
	      if (pwmDir[2]) orthoSuspensionDir(BACKWARD, &motorpwm[2]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[2]);
	      if (pwmDir[3]) orthoSuspensionDir(FORWARD, &motorpwm[3]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[3]);
	    }

	    break;	

	  case LEFT:
	  {
	      int16_t pwmCorrecion[4];
	      int8_t pwmDir[4];
		  
  // 	    correction = 0;
	      pwmCorrecion[0] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed + correction + correction2;
	      pwmCorrecion[1] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed + correction - correction2;
	      pwmCorrecion[2] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed - correction + correction2;
	      pwmCorrecion[3] = NORMAL_LATERAL_SPEED + parameters.offset[0] + lateralSpeed - correction - correction2;
	  
  //  	    Serial.print("Correction ");
  //  	    Serial.println(lateralSpeed);
	      for (i=0; i<4; i++)
	      {
		  pwmDir[i] = 1;
		  if (pwmCorrecion[i] < NORMAL_LATERAL_SPEED)
		  {
  // 		  pwmCorrecion[i] = NORMAL_LATERAL_SPEED + max(NORMAL_LATERAL_SPEED - pwmCorrecion[i], 0);
		    pwmDir[i] = 0;
		  } 
		  if (pwmCorrecion[i] > 200)
		      pwmCorrecion[i] = 200;
		  motorpwm[i].pwm = pwmCorrecion[i];
	      }
	      
	      if (pwmDir[0]) orthoSuspensionDir(FORWARD, &motorpwm[0]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[0]);            
	      if (pwmDir[1]) orthoSuspensionDir(BACKWARD, &motorpwm[1]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[1]);
	      if (pwmDir[2]) orthoSuspensionDir(FORWARD, &motorpwm[2]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[2]); 
	      if (pwmDir[3]) orthoSuspensionDir(BACKWARD, &motorpwm[3]);
	      else orthoSuspensionDir(BRAKE, &motorpwm[3]);
	      
	    }

	    break;	

	default:
	  for (i=0; i<4; i++)
	  {
	    motorpwm[i].pwm = 0;
	  }
	  orthoSuspensionRun(dir, 0);
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

