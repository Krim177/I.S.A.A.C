#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <arm_math.h>
#include "datastructures.h"
#include "pubsub.h"
#include "config.h"



// static float RotationMatrix[4][4];

float q0 = 1.0f, q1 = 0.0, q2 = 0.0, q3 = 0.0;

float q0q0 = 1.0;
float q0q1 = 0.0;
float q0q2 = 0.0;
float q0q3 = 0.0;
float q1q1 = 0.0;
float q1q2 = 0.0;
float q1q3 = 0.0;
float q2q2 = 0.0;
float q2q3 = 0.0;
float q3q3 = 0.0;


static QueueHandle_t mupGyroAccQueue;
static QueueHandle_t attitudeQueue;

static bool bInit = false;
static void attitudeTask(void* args);
TaskHandle_t attitudeHandle;

static uint8_t attitudePub = 0;


#define BETA_DEF     0.01f    // 2 * proportional gain
float beta = BETA_DEF;     // 2 * proportional gain (Kp)

void postCompute(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my);

void resetAttitude()
{
   q0 = 1.0f, q1 = 0.0, q2 = 0.0, q3 = 0.0;

   q0q0 = 1.0;
   q0q1 = 0.0;
   q0q2 = 0.0;
   q0q3 = 0.0;
   q1q1 = 0.0;
   q1q2 = 0.0;
   q1q3 = 0.0;
   q2q2 = 0.0;
   q2q3 = 0.0;
   q3q3 = 0.0;
   
   postCompute(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
   publish(attitudePub);
}


static void Madwick(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
   
    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt; 
}

void postCompute(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my)
{
  
   // Normalise quaternion
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q0q0 = q0*q0;
    q0q1 = q0*q1;
    q0q2 = q0*q2;
    q0q3 = q0*q3;
    q1q1 = q1*q1;
    q1q2 = q1*q2;
    q1q3 = q1*q3;
    q2q2 = q2*q2;
    q2q3 = q2*q3;
    q3q3 = q3*q3;
/*
    RotationMatrix[0][0] = 2*q0q0 - 1  + 2*q1q1;
    RotationMatrix[1][0] = 2*(q1q2 - q0q3);
    RotationMatrix[2][0] = 2*(q1q3 + q0q2);
    RotationMatrix[0][1] = 2*(q1q2 + q0q3);
    RotationMatrix[1][1] = 2*q0q0 - 1  + 2*q2q2;
    RotationMatrix[2][1] = 2*(q2q3 - q0q1);
    RotationMatrix[0][2] = 2*(q1q3 - q0q2);
    RotationMatrix[1][2] = 2*(q2q3 + q0q1);
    RotationMatrix[2][2] = q0q0 +q1q1 - q2q2 - q3q3;

 */
    Attitude_Data sensor;
    
//     float x = 2 * (q1q3 - q0q2);
//     float y = 2 * (q0q1 + q2q3);
//     float z = q0q0 - q1q1 - q2q2 + q3q3;
    
//     if (x>1) x=1;
//     if (x<-1) x=-1;
    
    
    sensor.yaw = atan2f(2*(q0q3 + q1q2), q0q0 + q1q1 - q2q2 - q3q3);
//     sensor.pitch = asinf(x); //Pitch seems to be inverted
//     sensor.roll = atan2f(y, z);
    

//     sensor.intertiaAx = (RotationMatrix[0][0] * ax + RotationMatrix[1][0] * ay + RotationMatrix[2][0] * az);
//     sensor.intertiaAy = (RotationMatrix[1][0] * ax + RotationMatrix[1][1] * ay + RotationMatrix[2][1] * az);
//     sensor.intertiaAz = (RotationMatrix[2][0] * ax + RotationMatrix[1][2] * ay + RotationMatrix[2][2] * az);
//  
    sensor.intertiaAx = deadband(ax, 0.04);
    sensor.intertiaAy = deadband(ay, 0.04);
    sensor.intertiaAz = deadband(az, 0.04);
//     sensor.gyroX = gx;
//     sensor.gyroY = gy;
    sensor.gyroZ = gz;
    sensor.heading = sensor.yaw;
    
//     if (my != 0.0f)
    {
//        float heading = atan2f(my, mx) * 180 / PI;
//        Serial.print("MAG ");
//        Serial.println(heading);
//        Serial.println(sensor.yaw);
    /*   Serial.print(",");
      Serial.print(sensor.pitch);
      Serial.print(",");
      Serial.print(sensor.roll);  
    */
//        Serial.print("Heading ");
//        Serial.println(sensor.heading);  	
    }
    xQueueOverwrite(attitudeQueue, &sensor);
}

void attitudeInit()
{
    if (bInit == false)
    {
	xTaskCreate(attitudeTask, "ATTITUDE", configMINIMAL_STACK_SIZE, NULL, 3, &attitudeHandle);
	bInit = true;
	mupGyroAccQueue = xQueueCreate(1, sizeof(Mpu_Data));
	attitudeQueue     = xQueueCreate(1, sizeof(Attitude_Data));

	registerSubscriber(MPU_DATA, attitudeHandle, mupGyroAccQueue);
	attitudePub = registerPublisher(ATTITUDE_DATA, sizeof(Attitude_Data), attitudeQueue);      
    }
}

bool attitudeTest()
{
    return bInit;
}


void attitudeTask(void* args)
{
    Mpu_Data gyroAcc, gyroAccTemp;
    uint32_t publisherId;
    float dt;
    float recipNorm;
    

    while (true)
    {
        if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
        {
	  // 0.0011375
	  dt = 0.001025;	
	  gyroAcc.mag.x = gyroAcc.mag.y = gyroAcc.mag.z  = 0.0f;
	  gyroAcc.gyro.x = gyroAcc.gyro.y = gyroAcc.gyro.z = 0.0f;
	  gyroAcc.acc.x = gyroAcc.acc.y = gyroAcc.acc.z = 0.0f;
	  gyroAccTemp.acc.x = gyroAccTemp.acc.y = gyroAccTemp.acc.z = 0.0f;
	  if (xQueueReceive(mupGyroAccQueue, &gyroAcc, ( TickType_t ) 0 )  == pdTRUE)
	  {
	    gyroAcc.gyro.x *= PI / 180;
	    gyroAcc.gyro.y *= PI / 180;
	    gyroAcc.gyro.z *= PI / 180;

	    recipNorm = invSqrt(gyroAcc.acc.x * gyroAcc.acc.x + gyroAcc.acc.y * gyroAcc.acc.y + gyroAcc.acc.z * gyroAcc.acc.z);
	    gyroAcc.acc.x = gyroAcc.acc.x * recipNorm;
	    gyroAcc.acc.y = gyroAcc.acc.y * recipNorm;
	    gyroAcc.acc.z = gyroAcc.acc.z * recipNorm;

	  }    
  /*     	MahonyAHRSupdate(gyroAcc.gyro.x, gyroAcc.gyro.y, gyroAcc.gyro.z, 
			      gyroAcc.acc.x,  gyroAcc.acc.y,  gyroAcc.gyro.z,
			      gyroAcc.mag.x, gyroAcc.mag.y, gyroAcc.mag.z, dt);
*/
	  Madwick(gyroAcc.gyro.x, gyroAcc.gyro.y, gyroAcc.gyro.z, 
				  gyroAcc.acc.x,  gyroAcc.acc.y,  gyroAcc.acc.z, dt);
				  
	  
	  
	  postCompute(gyroAcc.gyro.x, gyroAcc.gyro.y, gyroAcc.gyro.z,
		      gyroAcc.rawacc.x,  gyroAcc.rawacc.y,  gyroAcc.rawacc.z, 
		      gyroAcc.mag.x, gyroAcc.mag.y);
//  	  Serial.println(xTaskGetTickCount());

	  publish(attitudePub);

	
	}
 	   
    }
}
