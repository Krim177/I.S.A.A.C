#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <EEPROM.h>
#include <SPI.h>


#include "led.h"
#include "config.h"
#include "autopilot.h"
#include "virtualmachine.h"
#include "memory.h"
#include "robot.h"


#define ADDRESS_ID 0
#define ROBOT_TYPE 1
#define MOTOR_LATERAL_OFFSET 2
#define MOTOR_OFFSET 3
#define MOTOR_PIDSPEED 4
#define MOTOR_PIDLSPEED 16
#define MOTOR_PIDROTATION 28
#define MOTOR_PIDCORRECTION 40


SemaphoreHandle_t spiSemaphore;
SemaphoreHandle_t spiTransactionSemaphore;
Parameters parameters;

uint8_t robotType = ROBOT_TYPE;
uint8_t myId = 2;
uint8_t systemReady = 0;
volatile uint16_t errorCode = 0;

uint16_t getErrorCode()
{
    return errorCode;
}


uint8_t getId()
{
    return myId;
}

uint8_t getRobotType()
{
    return robotType;
}

int32_t readInteger(uint16_t address)
{   
    int32_t i;
    uint8_t buffer[4];
    buffer[3] = EEPROM.read(address);
    buffer[2] = EEPROM.read(address+1);
    buffer[1] = EEPROM.read(address+2);
    buffer[0] = EEPROM.read(address+3);
    memcpy(&i, buffer, 4);
    return i;
}

void writeInteger(uint16_t address, int value)
{   
    uint8_t buffer[4];
    memcpy(buffer, &value, 4);
    EEPROM.write(address, buffer[3]);
    EEPROM.write(address+1, buffer[2]);
    EEPROM.write(address+2, buffer[1]);
    EEPROM.write(address+3, buffer[0]);
}

void raedParameter()
{
    parameters.offset[0] = EEPROM.read(MOTOR_LATERAL_OFFSET);
    parameters.offset[1] = EEPROM.read(MOTOR_OFFSET);
    
    parameters.speed_p = readInteger(MOTOR_PIDSPEED) ;
    parameters.speed_i = readInteger(MOTOR_PIDSPEED+4);
    parameters.speed_d = readInteger(MOTOR_PIDSPEED+8);

    parameters.lspeed_p = readInteger(MOTOR_PIDLSPEED);
    parameters.lspeed_i = readInteger(MOTOR_PIDLSPEED+4);
    parameters.lspeed_d = readInteger(MOTOR_PIDLSPEED+8);

    
    parameters.rotation_p = readInteger(MOTOR_PIDROTATION);
    parameters.rotation_i = readInteger(MOTOR_PIDROTATION+4);
    parameters.rotation_d = readInteger(MOTOR_PIDROTATION+8);
                        
                                                                
    parameters.correction_p = readInteger(MOTOR_PIDCORRECTION);
    parameters.correction_i = readInteger(MOTOR_PIDCORRECTION+4);
    parameters.correction_d = readInteger(MOTOR_PIDCORRECTION+8);
}

int main() {

    //EEPROM.write(ADDRESS_ID, 5);
    //EEPROM.write(ROBOT_TYPE, CAR);
    myId = EEPROM.read(ADDRESS_ID);
    if (myId > 6)
      myId = 6;

    robotType = EEPROM.read(ROBOT_TYPE);
    raedParameter();    

                                                                 
    
    pinMode(13, OUTPUT);
   
#if FLOW_TASK == 1 || DWM_TASK == 1      
    SPI.begin();    
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);
    digitalWrite(10, HIGH);
    digitalWrite(9, HIGH);

    spiSemaphore = xSemaphoreCreateMutex();
    spiTransactionSemaphore = xSemaphoreCreateMutex();
#endif
    
    Serial.begin(115200);
    pubsubInit();    
    mpuInit();
    attitudeInit();
    
#if FLOW_TASK == 1   
    flowInit();
#endif
    
#if DWM_TASK == 1  
     dwmInit();
#endif
    
    stateInit();
  
#if UART_TASK == 1
    uartCommandInit();      
    memoryInit();
#endif  
 
  pathPlanningInit();

#if NRF_TASK == 1
    nrfInit();
#endif 
     virtualMachineInit();
     autopilotInit();
     ldmInit();   
     robotInit();
     
///////// Testing
    if (!pubsubTest())
    {
       errorCode = 1;
    }
    
    if (!mpuTest())
    {
      errorCode |= 1 << 1;
    }
    
    if (!attitudeTest())
    {
      errorCode |= 1 << 2;
    }
 
#if FLOW_TASK == 1
    if (!flowTest())
    {
        errorCode |= 1 << 3;    
    }
#endif
        
#if DWM_TASK == 1
    if (!dwmTest())
    {
	errorCode |= 1 << 4;
    }
#endif

     

#if NRF_TASK == 1
    if (!nrfTest())
    {
       errorCode |= 1 << 7;
    }
#endif
     
    if (!stateTest())
    {
       errorCode |= 1 << 5;
    } 
    
   if (!autopilotTest())
    {
        errorCode |= 1 << 12;
    }
	
    if (!robotTest())
    {
	    errorCode |= 1 << 13;
    }
     
    if (!ldmTest())
    {
       errorCode |= 1 << 6;
    }

    if (!pathPlanningTest())
    {
       errorCode |= 1 << 9;
    }

//      if (!virtualMachineTest())
//  	errorCode |= 1 << 11;
    
  
   
#if UART_TASK == 1
    if (!uartCommandTest() || !memoryTest())
    {
        errorCode |= 1 << 8;
    }
#endif    

     
    if (errorCode)   
      ledInit(); 
    

    delay(1000);

    // start scheduler, main should stop functioning here
    vTaskStartScheduler();

    for(;;);

    return 0;
}
