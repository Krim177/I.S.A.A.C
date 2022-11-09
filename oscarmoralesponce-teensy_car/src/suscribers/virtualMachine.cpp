#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "config.h"
#include "robot.h"
#include "autopilot.h"
#include "pathPlanning.h"
#include "state.h"


static LDMap     ldmap;
bool blocking = false;
static uint32_t clock = 0;
static int16_t distanceToTraverse;
static int8_t errorCode;

float originalPosX;
float originalPosY;
static Quaternion desiredHeading;
static Quaternion actualHeading;
static int16_t diffAngle;
static int16_t heading;

uint8_t modeController;


TaskHandle_t virtualMachineHandler;
TaskHandle_t sensorHandler;

static QueueHandle_t externalSensorQueue;
static QueueHandle_t stateQueue;
static QueueHandle_t externalCommandQueue;

static void virtualMachineTask(void* args);
static void externalSensorTask(void* args);
static void uartExternalCommandSend(void* args);

extern uint8_t systemReady;

enum {BOOTING = 0, INIT = 1,  RUNNING, HOLDING};

static uint8_t globalState = 0;
uint8_t mpu_state = RUNNING;


TIOAAlgorithm tioaAlgorithm;


void virtualMachineInit()
{
	xTaskCreate(virtualMachineTask, "VM", configMINIMAL_STACK_SIZE, NULL, 3, &virtualMachineHandler);

#if	SENSORS ==  EXTERNAL_SENSOR
	xTaskCreate(externalSensorTask, "ES", configMINIMAL_STACK_SIZE, NULL, 3, &sensorHandler);
	externalSensorQueue = xQueueCreate(1, sizeof(UARTSensor));
#endif
	stateQueue = xQueueCreate(1, sizeof(LDMap));
// 	attitudeQueue = xQueueCreate(1, sizeof(Attitude_Data));
		
	registerSubscriber(UART_SENSOR, sensorHandler, externalSensorQueue);
// 	registerSubscriber(ATTITUDE_DATA, sensorHandler, attitudeQueue);
 	registerSubscriber(STATE_DATA, sensorHandler, stateQueue);
#if SENSORS == EXTERNAL_SENSOR
	xTaskCreate(uartExternalCommandSend, "ECOMM", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	externalCommandQueue = xQueueCreate(1, sizeof(ExternalCommand));
#endif
	globalState = INIT;
	
#if APPLICATION == AEROSPACE
    tioaAlgorithm.init = onInitAerospace;
	tioaAlgorithm.newRound = onNewOrthoRobot;
	tioaAlgorithm.endOfRound = onEndOrthoRobot;
	tioaAlgorithm.OnExternalSensor = OnExternalSensorOrtho;
#elif APPLICATION == WAREHOUSE_ROBOT	
	tioaAlgorithm.init = onInitAerospace;
	tioaAlgorithm.newRound = onNewOrthoRobot;
	tioaAlgorithm.endOfRound = onEndOrthoRobot;
	tioaAlgorithm.OnExternalSensor = OnExternalSensorOrtho;
#elif APPLICATION == AUTONOMOUS_CAR
	tioaAlgorithm.init = onInitCar;
	tioaAlgorithm.newRound = onNewCarRobot;
	tioaAlgorithm.endOfRound = onEndCarRobot;
	tioaAlgorithm.OnExternalSensor = OnExternalSensorCar;
#endif  
  	
}

void sendTaskToExternalComputer(uint8_t code, uint8_t global, uint8_t local)
{
	ExternalCommand command;
	command.qrcode = code;
	command.globalState = global;
	command.localState = local;
	
	xQueueOverwrite(externalCommandQueue, &command);
}

static void uartExternalCommandSend(void *param)
{
	uint8_t *data;
	uint8_t ch;
	uint8_t i;
	ExternalCommand command;

	while (true)
	{
		xQueueReceive(externalCommandQueue, &command, portMAX_DELAY);
		data = (uint8_t *)&command;
		Serial.write(UART_FIRST_BYTE);
		Serial.write(UART_SECOND_BYTE);
		Serial.write(sizeof(command));
		ch = 0;
		for (i = 0; i < sizeof(ExternalCommand); i++)
		{
			ch = (ch + data[i]) % 256;
			Serial.write(data[i]);
		}
		Serial.write(ch);
		Serial.write("\n");
	}
}


void externalSensorTask(void* args)
{
    uint32_t publisherId;
    LDMap ldmapTemp;
    while (true)
    {
      if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
      {
	switch (publisherId)
	{
	  case UART_SENSOR:
	      UARTSensor uartSensorReading;

	      if (xQueueReceive(externalSensorQueue, &uartSensorReading, (TickType_t)0) == pdTRUE)
	      {
		      tioaAlgorithm.OnExternalSensor(&uartSensorReading, &ldmap, &globalState);
	      }

	      break;

	  case STATE_DATA:
	      if (xQueueReceive(stateQueue, &ldmapTemp, (TickType_t)0) == pdTRUE)
	      {
		 	 memcpy(&ldmap, &ldmapTemp, sizeof(LDMap));
	
	      }
	      break;
	   }
	 }
     }
}

void virtualMachineTask(void* args)
{
	UARTCommand command;
	command.maxTime = 1000;
	command.speed = 0;
	command.orientation = 0;
	command.distance = 0;
	command.dir = STANDBY;
	command.stop = false;
	command.stopCondition = NULL;
	command.adaptingOrientation = NULL;
	
	tioaAlgorithm.init();	
	portTickType lastWakeTime = xTaskGetTickCount();
	while (!systemReady)
	{
		vTaskDelayUntil(&lastWakeTime, 1 / portTICK_RATE_MS);
		if (!blocking)
		{
		  blocking = true;
		  setInstruction(&command);
		}
		
	}
	while (true)
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
		command.speed = 0;
		command.distance = 0;
		command.dir = STANDBY;
		command.stop = false;
		command.stopCondition = NULL;
		if (!blocking)
		{
			clock++;
			// Start a new round
			blocking = true;
			switch (globalState)
			{
			  case BOOTING:  
			    ekf_reset();

// 			    tioaAlgorithm.newRound(&command, &ldmap, globalState, clock);	
			    if (clock > 2000) 
					globalState = INIT;
			    break;
			  default:
			    	
 #if CONTROLLER == EXTERNAL_CONTROLLER
   			    getCommand(&command);
 			    setDistanceToTraverse(command.distance);
			    setAngleToRotate(command.orientation);
#elif CONTROLLER == HYBRID_CONTROLLER
			    getCommand(&command);
			    if (modeController == EXTERNAL_CONTROL)
			    {
			        setDistanceToTraverse(command.distance);
			        setAngleToRotate(command.orientation);
			    }
			    else 
			    {
			      tioaAlgorithm.endOfRound(&ldmap, &globalState, clock, errorCode, 0, 0);
			      memset(&command, 0, sizeof(command));
			      tioaAlgorithm.newRound(&command, &ldmap, globalState, clock);
			    }
#else
    			    tioaAlgorithm.endOfRound(&ldmap, &globalState, clock, errorCode, 0, 0);
     			    tioaAlgorithm.newRound(&command, &ldmap, globalState, clock);	
#endif
			    break;
			} 
			setInstruction(&command);
		} 
	}

	
}


void virtualComplete(uint8_t code, int16_t distance, int16_t angle)
{
    errorCode = code;
//     tioaAlgorithm.endOfRound(&ldmap, &globalState, clock, code, distance, angle);
}


void setDistanceToTraverse(uint16_t distance)
{
    originalPosX = ldmap.position_x;
    originalPosY = ldmap.position_y;
    distanceToTraverse = distance;
    heading = ldmap.heading;
}


void setAngleToRotate(int16_t desiredAngle)
{
     float desiredAngle2 = ldmap.heading + desiredAngle; 
     if (desiredAngle2> 180) 
	desiredAngle2 = desiredAngle2 - 360;
     if (desiredAngle2 < -180) 
	desiredAngle2 = 360 + desiredAngle2;
     
     diffAngle = desiredAngle2;

     initQuaternion(&desiredHeading, desiredAngle2  * PI/180.0f, 0.0f, 0.0f);
}

bool distanceCompleted()
{
    float dx = originalPosX - ldmap.position_x;
    float dy = originalPosY - ldmap.position_y;
    int16_t dist = distanceToTraverse - (int16_t)sqrt(dx*dx + dy*dy);
      Serial.print("Distance to traverse ");
      Serial.println(dist);
    return dist <=0;
    
}

int16_t distanceTraversed()
{
    float dx = originalPosX - ldmap.position_x;
    float dy = originalPosY - ldmap.position_y;
    int16_t dist = (int16_t)sqrt(dx*dx + dy*dy);
    return dist;
}

bool rotationCompleted()
{
    return abs(diffAngle) < 2;
}

/*********************************************************************************************
*  moveForward Move the car forward
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     dir: diretion
*     bool condition: if true, the car will brake
*     int16_t speed: speed to move. The default is 15
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t moveRobot(CarCommand *command, uint16_t dir, bool condition, int16_t speed,  bool stop, int16_t orientation)
{    
    float dx = originalPosX - ldmap.position_x;
    float dy = originalPosY - ldmap.position_y;
    int16_t dist = distanceToTraverse - (int16_t)sqrt(dx*dx + dy*dy);
    if (dist <= 0 || condition)  
    {				
        command->dir = BRAKE;
        command->stop = true;
        return COMPLETED_TIOA;	
    }
    else
    {
// 	heading += orientation;
        command->dir = dir;
        command->speed = speed;
        command->distance = max(min(10, dist), 3);
	command->orientation = orientation;
// 	Serial.print("Moving ");
// 	Serial.print(heading);
	if (dist < 10)
	  command->stop = true;
	else
	  command->stop = stop;
    } 
    return INCOMPLETED_TIOA;
}
    
/*********************************************************************************************
*  moveForward Move the car forward
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     bool condition: if true, the car will brake
*     int16_t speed: speed to move. The default is 15
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t moveForward(CarCommand *command, bool condition, int16_t speed,  bool stop, int16_t orientation)
{
    return moveRobot(command, FORWARD, condition, speed, stop, orientation);
}

/*********************************************************************************************
*  moveBackward Move the car backward
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     bool condition: if true, the car will brake
*     int16_t speed: speed to move. The default is 15
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t moveBackward(CarCommand *command, bool condition, int16_t speed, bool stop, int16_t orientation)
{
    return moveRobot(command, BACKWARD, condition, speed, stop, orientation);
}


/*********************************************************************************************
*  moveLeft Move the car left (front is where the autopilot is located)
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     bool condition: if true, the car will brake
*     int16_t speed: speed to move. The default is 15
*     int8_t minDistance: Distance to move in each step. The default value is 5
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t moveLeft(CarCommand *command, bool condition, int16_t speed, bool stop, int16_t orientation)
{ 
    return moveRobot(command, LEFT, condition, speed, stop, orientation);
}


/*********************************************************************************************
*  moveRight Move the car left (front is where the autopilot is located)
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     bool condition: if true, the car will brake
*     int16_t speed: speed to move. The default is 15
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t moveRight(CarCommand *command,  bool condition, int16_t speed, bool stop, int16_t orientation)
{
     return moveRobot(command, RIGHT, condition, speed, stop, orientation);
}

uint8_t rotateCar(CarCommand *command,  bool stop)
{
    Quaternion diffQ;
    initQuaternion(&actualHeading, ldmap.heading * PI/180.0f, 0.0f, 0.0f);
    diff(&actualHeading, &desiredHeading, &diffQ);
    diffAngle = getAngleFromQuaternion(&diffQ) * 180.0f / PI;
    if (abs(diffAngle) <= 1)
    {				
        command->dir = BRAKE;
        command->stop = true;
        return COMPLETED_TIOA;	
    }
    else
    {
        command->dir = ROTATE;
        command->speed = 0;
		command->distance = 0;
		
	// 	Serial.print("Rotating ");
		if (diffAngle > 0)
		{
	// 	  Serial.println(min(10, diffAngle));
		command->orientation = ldmap.heading + max(min(10, diffAngle), 3);
		}
		else 	  
		{
	// 	  Serial.println(max(-10, diffAngle));
		command->orientation = ldmap.heading + min(max(-10, diffAngle), -3);
		}
		command->stop = stop;
    } 
    return INCOMPLETED_TIOA;
}

/*********************************************************************************************
*  rotate Rotate the car 
* Params: 
*     CarCommand *command: The command to execute for the CarCommand
*     bool stop: if true, the car will stop after minDistance, otherwise it does not apply brakes.
*******************************************************************************************/
uint8_t rotate(CarCommand *command,  bool stop)
{
    return rotateCar(command, stop);
}
