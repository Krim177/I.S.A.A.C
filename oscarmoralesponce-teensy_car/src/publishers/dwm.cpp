
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <DW1000.h>
#include "pubsub.h"
#include "arm_math.h"
#include "dwm.h"
#include "datastructures.h"


#define CHANNEL_5 5
#define CHANNEL_2 2
#define TX_PULSE_FREQ_16MHZ 0x01
#define TRX_RATE_6800KBPS 0x02
#define TX_PREAMBLE_LEN_128 0x05
#define FRAME_LENGTH_NORMAL 0x00
#define PREAMBLE_CODE_16MHZ_4 4

#define PREAMBLE_CODE_64MHZ_9 9
#define TX_PULSE_FREQ_64MHZ 0x02

const uint8_t MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};

// connection pins
const uint8_t PIN_RST = 16; // reset pin
const uint8_t PIN_IRQ = 15; // irq pin
const uint8_t PIN_SS = 9; // spi select pin

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define NUMBER_OF_PARTICIPANTS 4

#define SPEED_OF_LIGHT 299792458.0f
#define LOCODECK_TS_FREQ 499.2e6 * 128

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET * LOCODECK_TS_FREQ) / SPEED_OF_LIGHT // In radio tick


bool inTimeSlot = false;
uint8_t timeSlot = 0;
static uint8_t curr_seq = 0;
uint8_t idLeader = 1;

static QueueHandle_t dwmQueue;
TaskHandle_t sendFinalHandle;

uint8_t dwmId = 0;

bool sendingpoll = false;
bool sendingfinal = false;

static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

double tround1, treply1, treply2, tround2, tprop_ctn, tprop, distance;

static bool bInit = false;
static void dwmTask(void* args);

Distance_Data distance_data[LOCODECK_NR_OF_ANCHORS];
packet_t txPacket;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) reportPayload_t;


byte mode = 1;

void setRx()
{
      DW1000.newReceive();
      DW1000.setDefaults();
      DW1000.receivePermanently(true);
      DW1000.startReceive();
}

void sendMessage(byte type)
{ 
      DW1000.idle();
      txPacket.seq = 0;
      txPacket.payload[LPS_TWR_TYPE] = type;
      txPacket.payload[LPS_TWR_SEQ] = curr_seq;
      
      DW1000.newTransmit();
      DW1000.setDefaults();

      if (type == LPS_TWR_POLL)
      {
	  sendingpoll = true;
      }
      if (type == LPS_TWR_FINAL)
      {
	   sendingfinal = true;
      }
      DW1000.setData((byte *)&txPacket, MAC802154_HEADER_LENGTH + 2);
      DW1000.waitForResponse(true);
      DW1000.startTransmit();      

}

void txcallback() {
      
}

void rxReceiveTimeStamp()
{  
    if (sendingpoll)
    {
      DW1000.getReceiveTimestamp(&answer_rx);
      answer_rx.full -= ANTENNA_DELAY / 2.0f;
      DW1000.getTransmitTimestamp(&poll_tx);         
      poll_tx.full   += ANTENNA_DELAY / 2.0f;
      sendingpoll = false;
    }
    if (sendingfinal)
    {
      DW1000.getTransmitTimestamp(&final_tx);
      final_tx.full += ANTENNA_DELAY / 2.0f;
      sendingfinal = false;
    }
}


   		   

void rxcallback()
{  
     int dataLength =  0;
     packet_t rxPacket;
    
     
     dataLength = DW1000.getDataLength();
     if (dataLength == 0)
     {
 	 setRx();
	 return ;
     }
      
     DW1000.getData((byte*)&rxPacket, dataLength);
     switch(rxPacket.payload[LPS_TWR_TYPE]) {
	  case LPS_TWR_POLL:
	      if (rxPacket.sourceAddress[0] <= idLeader)
	      {
		idLeader = rxPacket.sourceAddress[0];
		timeSlot = idLeader;
		txPacket.destAddress[0] = rxPacket.destAddress[0];
	      }
	      break;
 	  case LPS_TWR_FINAL:
	      break;
	  case LPS_TWR_ANSWER:	
	      
	      if (rxPacket.payload[LPS_TWR_SEQ] == curr_seq)// && inTimeSlot)  
	      { 
	      
   		  xTaskNotify(sendFinalHandle, 0, eSetBits);
	      }
	      break;
	  case LPS_TWR_REPORT:
	    {
		
 		if (rxPacket.payload[LPS_TWR_SEQ] == curr_seq) // && inTimeSlot)  
 		{	
		    reportPayload_t *report = (reportPayload_t *)(rxPacket.payload+2);


		    memcpy(&poll_rx, &report->pollRx, 5);
		    memcpy(&answer_tx, &report->answerTx, 5);
		    memcpy(&final_rx, &report->finalRx, 5);
		    
		    

		    tround1 = answer_rx.low32 - poll_tx.low32;
		    treply1 = answer_tx.low32 - poll_rx.low32;
 		    tround2 = final_rx.low32 - answer_tx.low32;
 		    treply2 = final_tx.low32 - answer_rx.low32;

		    tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

		    distance = (tprop_ctn) * 0.0045454545455f - 0.4f; //(0.0046917639786159f);
// 		    distance *= 100;
// 		    tprop = tprop_ctn / LOCODECK_TS_FREQ;
// 		    distance = SPEED_OF_LIGHT * tprop;		      
  
		      if (distance > 0 && distance < 10000.0f)
		      {
			  int index = txPacket.destAddress[0]-1;  
			  float weight = distance_data[index].error * 0.1;
			  if (weight > 1.0) weight = 1.0;
			  distance_data[index].distance = (distance_data[index].distance * (1.0 - weight)) + (weight * distance);
// 			  Serial.print(txPacket.destAddress[0]); 
// 			  Serial.print(":\t"); 
//  			  Serial.print(distance_data[index].distance);
// 			  Serial.print("\t"); 
// 			  Serial.println(distance);
			  distance_data[index].error = 1;
		      }
		}
	      }
	      break;
     }
     setRx();
}

void sendFinal(void* args)
{
    uint32_t publisherId;
    while (true)	
    {

      if (xTaskNotifyWait(0xffffffff, 0xffffffff, &publisherId, portMAX_DELAY) == pdTRUE)
      {
	  vTaskDelay(1);
	  sendMessage(LPS_TWR_FINAL);    
      }
    }
}


void dwmInit()
{
    int i;

    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    
    DW1000.setAntenaDelay(0);
    
    
     // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(txcallback);
    DW1000.attachReceivedHandler(rxcallback);
    DW1000.attachReceiveTimestampAvailableHandler(rxReceiveTimeStamp);

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.enableMode(DW1000Class::MODE_SHORTDATA_FAST_ACCURACY);
    DW1000.setChannel(CHANNEL_2);      
  // //     DW1000.setTxPower(0x1F1F1F1Ful);
    DW1000.setPreambleCode(PREAMBLE_CODE_64MHZ_9);
      
    DW1000.commitConfiguration();
    
   
    
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
  
    MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
    txPacket.pan = 0xbccf;
    txPacket.destAddress[0] = txPacket.destAddress[1] = txPacket.destAddress[2] = 
			      txPacket.destAddress[3] = txPacket.destAddress[4] = txPacket.destAddress[5] = 0;
    txPacket.destAddress[6] = 0xcf;
    txPacket.destAddress[7] = 0xbc;
    
    txPacket.sourceAddress[0] = getId();
    txPacket.sourceAddress[1] = txPacket.sourceAddress[2] = txPacket.sourceAddress[3] = 
				txPacket.sourceAddress[4] = txPacket.sourceAddress[5] = 0;
    txPacket.sourceAddress[6] = 0xcf;
    txPacket.sourceAddress[7] = 0x01;
    
    idLeader = getId();
    
    for (i=0; i<LOCODECK_NR_OF_ANCHORS; i++)
    {
	     distance_data[i].error = 15;
	     distance_data[i].distance = 0.0f;
    }
//     if (DW1000.isValid())
    {
       dwmQueue  = xQueueCreate(1, sizeof(distance_data));
       dwmId = registerPublisher(DWM_DATA, sizeof(distance_data), dwmQueue);
       xTaskCreate(sendFinal, "sendFinal", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &sendFinalHandle);
       xTaskCreate(dwmTask, "DWM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
       
        bInit = true;
     }
     setRx();
}



bool dwmTest()
{
    return bInit;
}

void dwmTask(void* args)
{
  
    idLeader = getId();
    
    portTickType lastWakeTime = xTaskGetTickCount();
     vTaskDelayUntil(&lastWakeTime, 2000 / portTICK_RATE_MS);
    while(1) {
      
     vTaskDelayUntil(&lastWakeTime, 15 / portTICK_RATE_MS);
      
      	
      // |     timeSlot      |
      // | 0 | 1 | 2 | 3 | 4 |   Address determine the ranging
      txPacket.destAddress[0] = (txPacket.destAddress[0] + 1) % LOCODECK_NR_OF_ANCHORS;
      if (txPacket.destAddress[0] == 0) 
	timeSlot = (timeSlot + 1) % NUMBER_OF_PARTICIPANTS;
      if (timeSlot == getId() && !inTimeSlot)
      {
	curr_seq++;  // increase the sequence
	
// Simulation of 
//  	distance_data[i].distance = 0.86602540378;
//  	if ((rand() % 10) > 2)
//  	{
//  	  distance_data[i].error = 1;
//  	  distance_data[i].distance += (rand() % 10) / 100.0f;
//  	}
//  	else
// 	{
//  	    distance_data[i].error++;
//  	}
//  	i++;
// 	i %= LOCODECK_NR_OF_ANCHORS; 
//  	Serial.print("Dist ");
// 	Serial.print(distance_data[timeSlot].distance);
// 	Serial.print(" err ");
// 	Serial.println(distance_data[timeSlot].error);
// end of Simulation	
	
	inTimeSlot = true;
      }
      
      if (timeSlot != getId() && inTimeSlot)  // The timeSlot for getId() is completed
      {
	 inTimeSlot = false;
   	 xQueueOverwrite(dwmQueue, distance_data);
   	 publish(dwmId);
// 	 Serial.println("Publish");
      }
      if (inTimeSlot)
      {
	  if (txPacket.destAddress[0] > 0 &&  txPacket.destAddress[0] < LOCODECK_NR_OF_ANCHORS)
	  {
	    if (distance_data[txPacket.destAddress[0]-1].error < 255)
	      distance_data[txPacket.destAddress[0]-1].error++;
	    
 	    sendMessage(LPS_TWR_POLL);
	    
 	  }
      }
   }
}
