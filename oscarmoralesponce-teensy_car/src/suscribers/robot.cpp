#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <PWMServo.h>
#include <arm_math.h>


#include "config.h"
#include "HardwareSerial.h"
#include "robot.h"


#define MAX_ANGLE 130
#define MIN_ANGLE 50
#define CENTER    90

#define MAX_SPEED 180
#define MIN_SPEED 10
#define NEUTRAL_SPEED 90

#define MOTOR_PIN 3
#define STEERING_PIN 6


#define CALIBRATE 1


uint8_t completed = READY;
bool setRobotInstruction = false;
static bool bInit = false;
static void robotTask(void* args);
static void stateRobotTask(void* args);

// static QueueHandle_t stateQueue;
static QueueHandle_t xCommand = NULL;
float orientation = 0;
float totalDistance = 0;

arm_pid_instance_f32 pidSpeed;
arm_pid_instance_f32 pidLateralSpeed;
arm_pid_instance_f32 pidCorrection;
arm_pid_instance_f32 pidRotation;
static QueueHandle_t stateQueue;
static QueueHandle_t attitudeQueue;
static float yaw = 0.0f;




RobotModel robotModel;
TaskHandle_t robotHandle = NULL;
TaskHandle_t stateRobotHandle = NULL;
static LDMap     ldmap; 

extern uint8_t systemReady;
static SemaphoreHandle_t mutex;
extern Parameters parameters;


void setPIdParamenters()
{
    Serial.print("Seting Pid Parameters");
  pidSpeed.Kp = parameters.speed_p / 1000.0f; 
    pidSpeed.Ki = parameters.speed_i / 1000.0f;
    pidSpeed.Kd = parameters.speed_d / 1000.0f;
    
    pidLateralSpeed.Kp = parameters.lspeed_p / 1000.0f; 
    pidLateralSpeed.Ki = parameters.lspeed_i / 1000.0f;
    pidLateralSpeed.Kd = parameters.lspeed_d / 1000.0f;
    
    pidCorrection.Kp = parameters.correction_p / 1000.0f;
    pidCorrection.Ki = parameters.correction_i / 1000.0f; 
    pidCorrection.Kd = parameters.correction_d / 1000.0f; 
    
    pidRotation.Kp = parameters.rotation_p / 1000.0f; 
    pidRotation.Ki = parameters.rotation_i / 1000.0f; 
    pidRotation.Kd = parameters.rotation_d / 1000.0f;   
  
    arm_pid_init_f32(&pidSpeed, 1);
    arm_pid_init_f32(&pidLateralSpeed, 1);
    arm_pid_init_f32(&pidRotation, 1);
    arm_pid_init_f32(&pidCorrection, 1);
}

void robotInit()
{
    mutex = xSemaphoreCreateMutex();
    xTaskCreate(robotTask, "ROBOT", configMINIMAL_STACK_SIZE, NULL, 3, &robotHandle);
    xTaskCreate(stateRobotTask, "STR", configMINIMAL_STACK_SIZE, NULL, 4, &stateRobotHandle);
    
    bInit = true;
    xCommand = xQueueCreate(1, sizeof(UARTCommand));
    stateQueue  = xQueueCreate(1, sizeof(LDMap));
    attitudeQueue  = xQueueCreate(1, sizeof(Attitude_Data));
    
    registerSubscriber(STATE_DATA, stateRobotHandle, stateQueue);
    registerSubscriber(ATTITUDE_DATA, stateRobotHandle, attitudeQueue);

    completed = READY;
    
     switch (getRobotType())
     {
       case ROVERCAR:
          robotModel.init = roverInit;
          robotModel.calibrate = roverCalibrate;
          robotModel.setRun = roverRun;
          robotModel.setSpeed = roverSetActuator;
        break;
      case ORTHOCAR:
          robotModel.init = orthoInit;
          robotModel.setRun = orthoRun;
          robotModel.calibrate = orthoCalibrate;
          robotModel.setSpeed = orthoSetActuator;
        break;
      case ORTHOSUSPENSIONCAR:
          robotModel.init = orthoSuspensionInit;
          robotModel.setRun = orthoSuspensionRun;
          robotModel.calibrate = orthoSuspensionCalibrate;
          robotModel.setSpeed = orthoSetSuspensionActuator;
        break;
      case CAR:
          robotModel.init = carInit;
          robotModel.calibrate = carCalibration;
          robotModel.setRun = carRun;
          robotModel.setSpeed = carSetActuator;
      break;
    }
    
    robotModel.init();
    setPIdParamenters();
}

bool robotTest()
{
    return bInit;
}

void robotSetInstruction(RobotInstructionState *ins)
{
    UARTCommand command;
    command.maxTime= ins->maxMilliseconds>>2; 
    
    command.speed = ins->speed; 
    command.distance = ins->distance;
    command.orientation = ins->orientation; 
    command.dir = ins->currentInstruction;
    command.stop = ins->stop;
    command.stopCondition = ins->stopCondition;
    command.adaptingOrientation = ins->adaptingOrientation;

   
     

    if (xCommand != NULL)
      xQueueOverwrite( xCommand, &command );
}

uint8_t robotGetStatus()
{
     if (setRobotInstruction && completed == READY)
     {
        setRobotInstruction = false;
        return true;
     }
     return false;
}

float prevSpeed;

float performInstruction(UARTCommand *command, LDMap  *ldmap, float diffAngle)
{
    float speedX;
    float speedY;	
    float angle = 0;
#if FLOW_TASK == 1
     float32_t diffSpeed = (float32_t)(command->speed - (ldmap->speed));
    float commonSpeed = arm_pid_f32(&pidSpeed, diffSpeed);
     float lateralSpeed = arm_pid_f32(&pidLateralSpeed, diffSpeed);
#else
  float commonSpeed = (float)command->speed;
  float lateralSpeed = (float)command->speed;
#endif

    float correction = arm_pid_f32(&pidCorrection, diffAngle);
    float rotation = arm_pid_f32(&pidRotation, diffAngle);
    
    float acc = ldmap->speed - prevSpeed;
    prevSpeed = ldmap->speed;



    if (commonSpeed > 100)
      commonSpeed = 100;
    if (commonSpeed < -40)
      commonSpeed = -40;

    if (rotation > 90)
      rotation = 90;
    if (rotation < -90)
      rotation = -90;
    

    Serial.print(getRobotType());
    Serial.print(" : ");
    Serial.println(commonSpeed);
    robotModel.setSpeed(commonSpeed, lateralSpeed, rotation, correction, acc, command->dir, &speedX, &speedY, ldmap);
    setMotorSpeed(speedX, speedY, angle);   // this
    return correction;
}


void stateRobotTask(void* args)
{
    uint32_t publisherId;
    LDMap ldmaptemp;
    Attitude_Data attitudeData;
    while (true)
    {
      
	if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
	{
	    switch (publisherId)
	    {
		case  ATTITUDE_DATA: 	  
		  if (xQueueReceive(attitudeQueue, &attitudeData, ( TickType_t ) 0 ) == pdTRUE)
		  {
		      xSemaphoreTake(mutex, portMAX_DELAY);
		      yaw = attitudeData.yaw;
		      xSemaphoreGive(mutex);
		  }
		  break;
		case STATE_DATA:
		  if (xQueueReceive(stateQueue, &ldmaptemp, ( TickType_t ) 0 ) == pdTRUE)
		  {
	
		    memcpy(&ldmap, &ldmaptemp, sizeof(LDMap));
		  }
		  break;
	    }
	}
    }
}

float getDiffAngle(Quaternion *q)
{
    Quaternion  p, diffQ;
    xSemaphoreTake(mutex, portMAX_DELAY);
    float tempYaw = yaw;
    xSemaphoreGive(mutex);

    initQuaternion(&p, tempYaw, 0.0f, 0.0f);
    diff(&p, q, &diffQ);
    float diffAngle = getAngleFromQuaternion(&diffQ) * 180.0f / PI;
//     Serial.print(tempYaw);  
//     Serial.print(" DiffAngle ");
//     Serial.println(diffAngle);  
    return diffAngle;
}


void robotTask(void* args)
{
    UARTCommand command;
    Quaternion  q;
    
    int16_t   maxTime = 0;
    uint32_t clock = 0;
    
    float prevPosX = 0, prevPosY = 0;
    float diffAngle = 0;
    bool stopCar = true;
    
    portTickType lastWakeTime = xTaskGetTickCount();
    
    vTaskDelayUntil(&lastWakeTime, 1000 / portTICK_RATE_MS);
    while (!systemReady)
    {
         vTaskDelayUntil(&lastWakeTime, 100 / portTICK_RATE_MS);
    }
    
    while (true) 
    {
        clock++;
        vTaskDelayUntil(&lastWakeTime, M2T(RATE_ROBOT));
        if (completed == READY)
        {
            if (xQueueReceive(xCommand, &command, ( TickType_t ) 0 )  == pdTRUE)
            {
                completed = IN_PROGRESS;
                setRobotInstruction = true;

                maxTime = command.maxTime;
                orientation = command.orientation;	      
                if (orientation > 180) 
                    orientation = orientation - 360;
                if (orientation < -180) 
                    orientation = 360 + orientation;
          
                initQuaternion(&q, orientation * PI/180.0f, 0.0f, 0.0f);
                
                totalDistance = command.distance * command.distance;
                prevPosX = ldmap.position_x;
                prevPosY = ldmap.position_y;
                    
                

         /*     Serial.print("DD0,");
                Serial.print(parameters.offset[0]);
                Serial.print(",");
                Serial.print(parameters.offset[1]);
                Serial.print(",");
                Serial.print(pidSpeed.Kp);
                Serial.print(",");
                Serial.print(pidSpeed.Ki);
                Serial.print(",");
                Serial.print(pidSpeed.Kd);
                Serial.print(",");
                Serial.print(pidLateralSpeed.Kp);
                Serial.print(",");
                Serial.print(pidLateralSpeed.Ki);
                Serial.print(",");
                Serial.println(pidLateralSpeed.Kd);
                Serial.print("DD0a,");
                Serial.print(pidRotation.Kp);
                Serial.print(",");
                Serial.print(pidRotation.Ki);
                Serial.print(",");
                Serial.print(pidRotation.Kd);
                Serial.print(",");
                Serial.print(pidCorrection.Kp);
                Serial.print(",");
                Serial.print(pidCorrection.Ki);
                Serial.print(",");
                Serial.println(pidCorrection.Kd);

                Serial.print("DD4,");
                Serial.print(command.dir);
                Serial.print(",");
                Serial.print(command.distance);
                Serial.print(",");
                Serial.print(command.orientation);
                Serial.print(",");
                Serial.println(yaw);  
                */
                
                arm_pid_reset_f32(&pidCorrection);
                arm_pid_reset_f32(&pidSpeed);
                arm_pid_reset_f32(&pidRotation);
                arm_pid_reset_f32(&pidLateralSpeed);
                stopCar = command.stop;
                robotModel.setRun(BRAKE, 0);
              }
              else
              {
                if (stopCar)
                      robotModel.setRun(BRAKE, 0);
                      else
                      {
                      if (command.stopCondition != NULL && command.stopCondition())
                      {
                        command.speed = 0;
                        stopCar = true;
                        robotModel.setRun(BRAKE, 0);
                      }
                      else
                      {
                        if (command.speed > 0)
                          command.speed = command.speed >> 0;
                        else 
                          command.speed = 0;
                        diffAngle = getDiffAngle(&q);
                        performInstruction(&command, &ldmap, diffAngle);  
                      }
                }
              }
            }
            else 
            {    
                float dist = 0;

                
                float posX = ldmap.position_x - prevPosX;
                float posY = ldmap.position_y - prevPosY;
            
                dist =  (posX*posX) + (posY*posY);	
                diffAngle = getDiffAngle(&q);
                
                          
                  if (command.dir == FORWARD || command.dir == BACKWARD || command.dir == LEFT || command.dir == RIGHT)
                  {
                  if (dist  >= totalDistance  &&  completed != READY)
                  {
                          
                        /*  Serial.print("DD5,Completed,");
                          Serial.print(command.distance - sqrt(dist));
                          Serial.print(",");
                          Serial.println(diffAngle);
                          */
                
                    completed = READY;	      
                  virtualComplete(0, command.distance - totalDistance, 0);

                  }
                  }
                  if (command.stopCondition != NULL && command.stopCondition())
                  {
                        completed = READY;
                        virtualComplete(0, command.distance - totalDistance, 0);
                        command.speed = 0;
                        stopCar = true;
                        robotModel.setRun(BRAKE, 0);
                 /*       Serial.print("DD5,Completed,");
                          Serial.print(command.distance - sqrt(dist));
                          Serial.print(",");
                          Serial.println(diffAngle);  */

                  }
                  if (completed != READY && command.dir == ROTATE && abs(diffAngle) < 1 && ldmap.isRotating)
                  {
                    /*  Serial.print("DD5,Completed,");
                      Serial.print(command.distance - sqrt(dist));
                      Serial.print(",");
                      Serial.println(diffAngle);
                    */
                  completed = READY;
                  virtualComplete(0, command.distance - totalDistance, 0);
                  }
                  maxTime--;
                  if (command.dir == STANDBY && maxTime < (command.maxTime >> 1) && completed != READY) 
                  {
              /*          Serial.print("DD5,Completed,");
                      Serial.print(command.distance - sqrt(dist));
                      Serial.print(",");
                      Serial.println(diffAngle); */
                        completed = READY;
                        virtualComplete(0, command.distance + dist, 0);    
                        robotModel.setRun(BRAKE, 0);
                  }
                  if (completed != READY)
                    performInstruction(&command, &ldmap, diffAngle);
              
                  if (maxTime < 0 && completed != READY) 
                  {
                      float xsqrt = sqrt(dist);
                      float x = command.distance - xsqrt;
                   /*   Serial.print("DD5,Timeout,");
                      Serial.print(x);
                      Serial.print(",");
                      Serial.println(diffAngle); */
                      completed = READY;    // Exception
                      setOperationMode(AUTONOMOUS);
                if (0.7*command.distance >= x)
          //             if (0.4*command.distance <= xsqrt)
                {
                 // Serial.print(x);
                 // Serial.println("Timeout No Failure");
                  virtualComplete(0, x, 0);
                }
                else
                {
                  //Serial.print(x);
                  //Serial.println("Timeout Failure");
                  virtualComplete(1, x, 0);
                }
                robotModel.setRun(BRAKE, 0); 
              }  
        } 
    }   
}

