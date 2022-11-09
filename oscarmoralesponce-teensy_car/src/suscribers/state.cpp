/**
* state of the robot. 
*
* @author  Oscar Morales-Ponce
* @version 0.1
* @since   04-08-2019 
* 
* It computes a Kalman filter to determine the position and speed of the robot. 
* Publishes:
*	STATE_DATA 
* Subscribes: 	
* 	ATTITUDE_DATA: provides the attitude of the robot
* 	MPU_DATA: provides the readings of the MPU Gyro and Acce
* 	FLOW_DATA: provides the reading of the movement on X and Y (FlowDeck)
* 	DWM_DATA: provides the reading of position x,y,z from the DWM
* 
*/

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <Arduino.h>
#include <arm_math.h>
#include "pubsub.h"
#include "ekf.h"


#include "config.h"
#include "autopilot.h"


#define UNIT   2.0f
#define SQRT2  1.4142135623731f
#define DIAGONAL UNIT * SQRT2

#define CALIBRATE_FLOW 1
#define ROTATING_THRESHOLD 0.001

static bool bInit = false;
static void kalmanTask(void* args);
static void stateTask(void* args);


TaskHandle_t stateHandle = NULL;
TaskHandle_t kalmanHandle = NULL;

extern uint8_t systemReady;


uint8_t stateId;
float yaw = 0.0f;
#define FREQUENCY_KF   10  // milliseconds
#define DT	     FREQUENCY_KF/1000.0f

#define DWMSTDDEV 0.2
#define STATE_SIZE 4

#define X_STATE      0
#define Y_STATE      1
#define XDOT_STATE   2
#define YDOT_STATE   3


#define VALUE_SIZE 6

#define GPSX_VALUE  0
#define GPSY_VALUE  1
#define FLOWX_VALUE 2
#define FLOWY_VALUE 3
#define FLOWX_POS   4
#define FLOWY_POS   5



#define WEIGHT   0.0f


static QueueHandle_t attitudeQueue;
static QueueHandle_t flowQueue;
static QueueHandle_t dwmQueue;
static QueueHandle_t stateQueue;
static QueueHandle_t mpuGyroAccQueue;

ekf_t ekf;


// Variables for the Kalman Filter
float32_t X[STATE_SIZE];
float32_t x[STATE_SIZE];
float32_t F[STATE_SIZE][STATE_SIZE];
float32_t P[STATE_SIZE][STATE_SIZE];
float32_t Q[STATE_SIZE][STATE_SIZE];
float32_t R[VALUE_SIZE][VALUE_SIZE];
float32_t Y[VALUE_SIZE];
float32_t Z[VALUE_SIZE];
float32_t H[VALUE_SIZE][STATE_SIZE];

float32_t muX, muY, theta;
float32_t accX, accY, gyro;
uint8_t accSample = 0;
uint8_t motorSample = 0;
uint8_t flowSample = 0;
static SemaphoreHandle_t xSemaphore = NULL;
float posX = 0, posY = 0;
float velX= 0, velY= 0;
float flowPosX = 0;
float flowPosY = 0;


/* 
* ekf_reset: Initialize the variable of the Kalman filter to reinitialize. 
* The function has been left of the kalman filter so it can be customize 
*/
void ekf_reset()
{ 
    uint8_t i;
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    
    resetAttitude();
    arm_fill_f32(0.0f, &F[0][0], STATE_SIZE*STATE_SIZE);
    arm_fill_f32(0.0f, &P[0][0], STATE_SIZE*STATE_SIZE);
    arm_fill_f32(0.0f, &H[0][0], VALUE_SIZE*STATE_SIZE);
    
    arm_fill_f32(0.0f, &Q[0][0], STATE_SIZE*STATE_SIZE);
    arm_fill_f32(0.0f, x, STATE_SIZE);
    arm_fill_f32(0.0f, &R[0][0], VALUE_SIZE*VALUE_SIZE);
    arm_fill_f32(0.0f, &Z[0], VALUE_SIZE);
    arm_fill_f32(0.0f, &Y[0], VALUE_SIZE);
        
    
    for (i=0; i<STATE_SIZE; i++)
    {
        F[i][i] = 1.0f;
    }
    
    
    H[GPSX_VALUE][X_STATE] = 1.0f;
    H[GPSY_VALUE][Y_STATE] = 1.0f;
    H[FLOWX_VALUE][XDOT_STATE] = 1.0f;   
    H[FLOWY_VALUE][YDOT_STATE] = 1.0f;    
    
    H[FLOWX_POS][X_STATE] = 1.0f;   
    H[FLOWY_POS][Y_STATE] = 1.0f;    
    
#if DWM_TASK == 1
    Q[X_STATE][X_STATE] = 1000.0f;
    Q[Y_STATE][Y_STATE] = 1000.0f;
#else
    Q[X_STATE][X_STATE] = 0.0f;
    Q[Y_STATE][Y_STATE] = 0.0f;
#endif
    Q[XDOT_STATE][XDOT_STATE] = 100.0f;
    Q[YDOT_STATE][YDOT_STATE] = 100.0f;
    
    P[X_STATE][X_STATE] = 0.4f;
    P[Y_STATE][Y_STATE] = 0.4f;
    P[XDOT_STATE][XDOT_STATE] = 0.4f;
    P[YDOT_STATE][YDOT_STATE] = 0.4f;
    
   
    R[GPSX_VALUE][GPSX_VALUE] = 1000.0f;
    R[GPSY_VALUE][GPSY_VALUE] = 1000.0f;
    R[FLOWX_VALUE][FLOWX_VALUE] = 1000.0f;
    R[FLOWY_VALUE][FLOWY_VALUE] = 1000.0f;
    R[FLOWX_POS][FLOWX_POS] = 1000.0f;
    R[FLOWY_POS][FLOWY_POS] = 1000.0f;
    
    X[XDOT_STATE] = 0;
    X[YDOT_STATE] = 0;

    X[X_STATE] = 0;
    X[Y_STATE] = 0;
    
    accX = 0.0f;
    accY = 0.0f;
    accSample = 0;
        
    xSemaphoreGive( xSemaphore );
    
//      gyro = 0.0f;
}


/* 
* ekf_attitude_init: Initialize the matrices of the kalman filter. 
* @param type: the message  (RADIO_CHANNEL, RADIO_DATARATE, RADIO_ADHOC)
* @param param: The parameter of the message
*/
void ekf_attitude_init()
{
  
    xSemaphore = xSemaphoreCreateMutex();
    
    ekf_reset();
  
    arm_mat_init_f32(&ekf.H,  VALUE_SIZE, STATE_SIZE, &H[0][0]);
    
    arm_mat_init_f32(&ekf.F,  STATE_SIZE, STATE_SIZE, &F[0][0]);
    arm_mat_init_f32(&ekf.x,  STATE_SIZE, 1, &x[0]);	
    arm_mat_init_f32(&ekf.X,  STATE_SIZE, 1, &X[0]);
    arm_mat_init_f32(&ekf.P,  STATE_SIZE, STATE_SIZE, &P[0][0]);
    arm_mat_init_f32(&ekf.Q,  STATE_SIZE, STATE_SIZE, &Q[0][0]);
    arm_mat_init_f32(&ekf.R,  VALUE_SIZE, VALUE_SIZE, &R[0][0]);
    arm_mat_init_f32(&ekf.Y,  VALUE_SIZE, 1, &Y[0]);

      
    ekf_init(&ekf, STATE_SIZE, VALUE_SIZE);
    
  
}


/* 
* ekf_attitude_update: Execute a step ot the kalman filter. 
* @param accX: accumuation of the acceleration on X
* @param accY: accumuation of the acceleration on Y
* @param gyro: accumuation of the gyroscope
*/
void ekf_attitude_update(float32_t accX, float32_t accY, float32_t gyro)
{
       float32_t cosyaw = cos(yaw);
       float32_t sinyaw = sin(yaw);	
       
       float xdotstate = x[XDOT_STATE] * DT;
       float ydotstate = x[YDOT_STATE] * DT;
       
       X[X_STATE] = x[X_STATE] + (xdotstate * cosyaw + ydotstate * sinyaw); 
       X[Y_STATE] = x[Y_STATE] + (-xdotstate * sinyaw + ydotstate * cosyaw); 
       
//         Serial.print(xdotstate);
//         Serial.print(" , ");
//         Serial.print(ydotstate);
//         Serial.print(" , ");
//         Serial.print(X[X_STATE]);
//         Serial.print(" , ");
//         Serial.println(X[Y_STATE]);
//         Serial.print(", ");
//         Serial.println(accY);
      
      
      X[XDOT_STATE] = x[XDOT_STATE] + accX;  
      X[YDOT_STATE] = x[XDOT_STATE] + accY;  


    
      F[X_STATE][XDOT_STATE] = DT*cosyaw;
      F[X_STATE][YDOT_STATE] = -DT*sinyaw;
      F[Y_STATE][XDOT_STATE] = DT*sinyaw;
      F[Y_STATE][YDOT_STATE] = DT*cosyaw;
 
      
//       Serial.print(yaw * 180 / PI);
//         Serial.print(",  ");
//         Serial.print(Z[FLOWX_POS] * 100);
//         Serial.print(" = ");
//         Serial.print(X[X_STATE] * 100);
// 	Serial.print(", ");
// 	Serial.print(Z[FLOWY_POS] * 100);
//         Serial.print(" = ");
//         Serial.println(X[Y_STATE] * 100);

      
      Y[FLOWX_POS] = Z[FLOWX_POS]  - X[X_STATE];
      Y[FLOWY_POS] = Z[FLOWY_POS] - X[Y_STATE];  
      Y[GPSX_VALUE] = Z[GPSX_VALUE] - X[X_STATE];
      Y[GPSY_VALUE] = Z[GPSY_VALUE] - X[Y_STATE];
      Y[FLOWX_VALUE] = Z[FLOWX_VALUE]  - X[XDOT_STATE];
      Y[FLOWY_VALUE] = Z[FLOWY_VALUE] - X[YDOT_STATE];  
    
      ekf_update(&ekf);
}


/* 
* setMotorSpeed: accumulates the theoretical values of the motors. 
* @param motorX: The motor power to move forward 
* @param motorY: The motor power to move horizontal
* @param angle: the motor power to steer
*/
void setMotorSpeed(float motorX, float motorY, float angle)
{
    muX += motorX;
    muY += motorY;
    motorSample++;
    theta += angle;
}

/* 
* setMotorReset: initialize the theoretical values of the motors. 
*/
void setMotorReset()
{
    muX = 0;
    muY = 0;
    theta = 0;
}


/* 
* stateInit: Initialize the task of the state. It subscribes to
* ATTITUDE_DATA: provides the attitude of the robot
* MPU_DATA: provides the readings of the MPU Gyro and Acce
* FLOW_DATA: provides the reading of the movement on X and Y (FlowDeck)
* DWM_DATA: provides the reading of position x,y,z from the DWM
* The task register as publisher which publish the new state
*/
void stateInit()
{
    if (bInit == false)
    {
    	xTaskCreate(stateTask, "STATE", configMINIMAL_STACK_SIZE, NULL, 4, &stateHandle);
    	xTaskCreate(kalmanTask, "KALMAN", configMINIMAL_STACK_SIZE, NULL, 3, &kalmanHandle);
    	ekf_attitude_init();

	attitudeQueue  = xQueueCreate(1, sizeof(Attitude_Data));
	dwmQueue  = xQueueCreate(1, sizeof(Distance_Data)*LOCODECK_NR_OF_ANCHORS);
	flowQueue  = xQueueCreate(1, sizeof(Flow_Data));
	stateQueue  = xQueueCreate(1, sizeof(LDMap));
	mpuGyroAccQueue = xQueueCreate(1, sizeof(Mpu_Data));


    	
        registerSubscriber(ATTITUDE_DATA, stateHandle, attitudeQueue);
	registerSubscriber(MPU_DATA, stateHandle, mpuGyroAccQueue);
    #if DWM_TASK == 1
      	registerSubscriber(DWM_DATA, stateHandle, dwmQueue);
    #endif
    	
    #if FLOW_TASK == 1
    	registerSubscriber(FLOW_DATA, stateHandle, flowQueue);	
    #endif

	stateId = registerPublisher(STATE_DATA, sizeof(LDMap), stateQueue);
	
	if (stateId > 0)
	  bInit = true;
    }
}

/* 
* stateTest: return true if the task was correctly initialized. 
*/
bool stateTest()
{
    return bInit;
}

/* 
* kalmanTask: It executes at FREQUENCY_KF the Kalman filter and publish 
* the output o
*/
void kalmanTask(void* args)
{
    float prevx = 0, prevy = 0;
    uint32_t lastWakeTime = xTaskGetTickCount();
     while (!systemReady)
    {
         vTaskDelayUntil(&lastWakeTime, 10);
    }
    
    while (true)
    {
       vTaskDelayUntil(&lastWakeTime, FREQUENCY_KF / portTICK_RATE_MS);
//        if (accSample > 0)
//         {
//           accX /= (float)accSample;
//           accY /= (float)accSample;
//         }
//         else 
//         {
//           accX = 0;
//           accY = 0;
//         }
//     Serial.print("Z x ");
//   		    Serial.print(Z[FLOWX_VALUE]);
//   		    Serial.print(" Z y ");
//   		    Serial.println(Z[FLOWY_VALUE]);	
       xSemaphoreTake(xSemaphore, portMAX_DELAY);
       
       ekf_attitude_update(accX, accY, gyro);
       if  (ekf_step(&ekf))
       {
	   float speed;
           LDMap ldmap;
	   ldmap.id = getId();
// 	   ldmap.timestamp = xTaskGetTickCount();
	   ldmap.position_x = (x[X_STATE] * 100.0f);
 	   ldmap.position_y = (x[Y_STATE] * 100.0f);
// 	   ldmap.position_z = 0;
	   
	   
	   float diffX = prevx - x[X_STATE];
	   float diffY = prevy - x[Y_STATE];
	   
 	   speed = 1000 * sqrt(diffX*diffX + diffY*diffY) / FREQUENCY_KF;
	   prevx = x[X_STATE];
	   prevy = x[Y_STATE];
	   
	   
	   ldmap.speed = speed;
	   ldmap.heading =  (int16_t)(yaw * 180/ PI); //

// 	   Serial.println(speed);
//   	   Serial.print("State Speed ");
//   	   Serial.println(ldmap.speed);
  	   
// 	   Serial.print("HEADING ");
// 	   Serial.println(ldmap.heading);
       
//  	  Serial.print(x[YDOT_STATE] * 100);
//  	  Serial.print(" Pos ");
//  	  Serial.println(velY*100);
	    
       
// 	   ldmap.clock = geClock();
	   ldmap.operationMode = getOperationMode();
  	   ldmap.tioaState = getCooperativeState();
	   ldmap.membersInProgress = getMembersInProgress();
	   ldmap.membersInSync = getMembersInSync();
           ldmap.isRotating = abs(gyro) >= ROTATING_THRESHOLD; 
// 	   Serial.print("*G ");
// 	   Serial.println(ldmap.isRotating);
           
// 	   ldmap.messageData.param.qrCode = 0;
// 	   ldmap.messageData.distance = 0;
	   ldmap.messageData.Ydistance = 0;
	   
// 	   Serial.print("ID in State ");
// 	   Serial.println(ldmap.id);
	   
	  	   
//  	   if (ldmap.heading > 180)
//  	     ldmap.heading = 360 - ldmap.heading;
// 	   else
//   	   Serial.println(ldmap.heading);
//  	   Serial.print(", ");
//     	   Serial.print(ldmap.position_x);
//   	  Serial.print(",");
//   	  Serial.println(ldmap.position_y);
           ldmap.rssi = 0; 

           xQueueOverwrite(stateQueue, &ldmap);
	   publish(stateId);
 	}
 	xSemaphoreGive( xSemaphore );
 	
  	gyro =  theta = 0.0f;
 	accX = accY = 0.0f;
	accSample = 0;
	motorSample = 0;
	muX = 0.0;
	muY = 0.0;
	flowSample = 0;
	
	Z[GPSX_VALUE] = x[X_STATE];
        Z[GPSY_VALUE] = x[Y_STATE];
	Z[FLOWX_VALUE] = 0;
	Z[FLOWY_VALUE] = 0;
	
// 	Z[FLOWX_POS] = 0;
// 	Z[FLOWY_POS] = 0;

	R[GPSX_VALUE][GPSX_VALUE] = 10000.0f;
	R[GPSY_VALUE][GPSY_VALUE] = 10000.0f;
	R[FLOWX_VALUE][FLOWX_VALUE] = 10000.0f;
	R[FLOWY_VALUE][FLOWY_VALUE] = 10000.0f;
	R[FLOWX_POS][FLOWX_POS] = 10000.0f;
	R[FLOWY_POS][FLOWY_POS] = 10000.0f;	
	
	
       
  }
}


/* 
* stateTask: it computes when the data is received  from the subscription
* ATTITUDE_DATA: Gets the yaw
* MPU_DATA: acccumulate the accelerometer reading
* FLOW_DATA: computes the actual distance based on the pixel movements
* DWM_DATA: Execute a triangulation
*/
void stateTask(void* args)
{
    uint32_t publisherId;
    uint32_t lastWakeTime = xTaskGetTickCount();

    while (!systemReady)
    {
        vTaskDelayUntil(&lastWakeTime, 1000);
    }
    
    while (true)	
    {

      if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
      {
	 
	  switch (publisherId)
	  {
	    case MPU_DATA:
	      {
		  Mpu_Data gyroAcc;
		  if (xQueueReceive(mpuGyroAccQueue, &gyroAcc, ( TickType_t ) 0 )  == pdTRUE)
		  {
		     accX += deadband(gyroAcc.rawacc.x, 0.5f) * 0.160; 
		     accY += deadband(gyroAcc.acc.y, 0.1f) * 0.160;
		     
//   		      accSample++;
		  }
	      }
	      break;
	    case  ATTITUDE_DATA: 
 	      {	
      		  Attitude_Data attitudeData;
		  if (xQueueReceive(attitudeQueue, &attitudeData, ( TickType_t ) 0 ) == pdTRUE)
		  {
 		    gyro += attitudeData.gyroZ * 0.00108;
		    

		    yaw = attitudeData.yaw;
		  }
	      }
	      break;
	    case FLOW_DATA:
	      {  
  //  		float32_t dtFlow = (xTaskGetTickCount() - flowTime) / 1000.0f;
		    
		  Flow_Data flowData;
		  if (xQueueReceive(flowQueue, &flowData, ( TickType_t ) 0 )  == pdTRUE)
		  {
		    
		    float32_t cosyaw = cos(yaw);
		    float32_t sinyaw = sin(yaw);	
       
		
       
		    float distX = flowData.deltax * -0.00024625;
		    float distY = flowData.deltay * -0.00024625;
		    
// 		    float velX = distX / 33;
// 		    float velY = distY / 33;
		    
		    Z[FLOWX_POS] +=  distX * cosyaw + distY * sinyaw; 
		    Z[FLOWY_POS] += -distX * sinyaw + distY * cosyaw; 
		    
		    Z[FLOWX_VALUE] = distX / 33; // Meters per second
		    Z[FLOWY_VALUE] = distX / 33; 
		    
//       		    Serial.print("Z x ");
//       		    Serial.print(velX * 1000);
//       		    Serial.print(" FlowY ");
//        		    Serial.println(velY * 1000);
		    
// 		     Serial.print("SPeed ");
// 		     Serial.println(accY* 1000);
// 		    accX = accY = 0.0f;
		    
		    R[FLOWX_VALUE][FLOWX_VALUE] = 0.005;
		    R[FLOWY_VALUE][FLOWY_VALUE] = 0.005;
		    R[FLOWX_POS][FLOWX_POS] = 0.005;
		    R[FLOWY_POS][FLOWY_POS] = 0.005;
		  }
	      }
	      break;
	    case DWM_DATA:
	    {
		Distance_Data distanceData[LOCODECK_NR_OF_ANCHORS+1];
    		float d[LOCODECK_NR_OF_ANCHORS];
    		float p[6], b[6];
		float error = 0.0f;

    		const float rsquare = DIAGONAL * DIAGONAL;
    		const float r2 = 2*DIAGONAL;
    		int i;

		if (xQueueReceive(dwmQueue, distanceData, ( TickType_t ) 0 ) == pdTRUE)
		{
		    for (i=0; i<LOCODECK_NR_OF_ANCHORS; i++)
		    {
			  if (distanceData[i].distance > 3 || distanceData[i].distance < -1)
			    break;
			  d[i] = distanceData[i].distance * distanceData[i].distance;
 			  error += distanceData[i].error / 40.0f;
		    }

		    /*
		   Serial.print(d[0]);
		   Serial.print(", ");
		   Serial.print(d[1]);
		   Serial.print(", ");
		   Serial.print(d[2]);
		   Serial.print(", ");
		   Serial.print(d[3]);
		   Serial.print(", ");
		   Serial.println(error);*/
		    if (!(d[0] == 0 || d[1] == 0 || d[2] == 0 || d[3] == 0))
		    {
		      p[0] =  (rsquare + d[0] - d[1])/r2;
		      p[1] =  (rsquare + d[0] - d[2])/r2;
		      p[2] =  (rsquare + d[0] - d[3])/r2;
		      p[3] =  (rsquare + d[1] - d[2])/r2;
		      p[4] =  (rsquare + d[1] - d[3])/r2;
		      p[5] =  (rsquare + d[2] - d[3])/r2;

		      b[0] = p[0] * SQRT2;
		      b[1] = p[1] * SQRT2;
		      b[2] = p[2] * SQRT2;
		      b[3] = UNIT - p[3] * SQRT2;
		  //     b[4] = DIAGONAL - p[4] * SQRT2;
		      b[5] = UNIT - p[5] * SQRT2;

		      Z[GPSX_VALUE] = (b[0] + b[5])/2.0f;
		      Z[GPSY_VALUE] = (b[0] - b[5])/2.0f;
//  		      Z[GPSZ_VALUE] = 100.0f * (b[2] - b[3])/2.0f;
		
// 		      Serial.print("Pos ");
// 		      Serial.print(Z[GPSX_VALUE]);
// 		      Serial.print(", ");
// 		      Serial.println(Z[GPSY_VALUE]);
		      
		      R[GPSX_VALUE][GPSX_VALUE] = error; // DWMSTDDEV / error;
		      R[GPSY_VALUE][GPSY_VALUE] = error; // DWMSTDDEV / error;
//  		      R[GPSZ_VALUE][GPSZ_VALUE] = DWMSTDDEV * error;
		    }
		}
	    }
	    break;
	  }
      }
  }
}
