#ifndef   __NRF51822_TASK__
#define   __NRF51822_TASK__


#include "datastructures.h"
#include "uartCommand.h"

#define RADIO_CHANNEL       0x01
#define RADIO_DATARATE      0x02
#define RADIO_ADDRESS       0x05
#define RADIO_CONTROL       0x07
#define RADIO_ADHOC        0x06





typedef struct _tCommonKnowledge {
//    uint32_t clock;
   int16_t position_x,position_y; //,position_z; // decimenter 
   int16_t speed; 
   int16_t heading;
   uint16_t membersInProgress;
   uint16_t membersInSync;
   uint8_t operationMode : 1;
   uint8_t propose : 1;
   uint8_t tioaState : 2;   
    uint8_t instruction : 4;
   int8_t  data[13];
} CommonKnowledge;

/* ESB Radio packet */
typedef struct esbPacket_s {
  /* Part that is written by the radio DMA */
  struct {
    uint8_t size;
    union {
      uint8_t s1;
      struct {
        uint8_t noack :1;
        uint8_t pid :2;
      };
    };
    union {
      uint8_t data[32];
      struct {
	uint8_t messageId; 
	uint8_t id :4;
	uint8_t state :4;
	uint16_t members;
	uint8_t leaderId;
	uint8_t time;
	union 
	{
	  uint8_t data[26];
	  //LDMap ldmap;
	};
      } msg;
    };
  } __attribute__((packed));
  uint8_t rssi;
  unsigned int crc;
} EsbPacket;

struct _tNRFPacket
{
    uint8_t type;
    uint8_t length;
    union
    {
      uint8_t data[32];
      struct {
	uint8_t messageId; 
	uint8_t id :4;
	uint8_t state :4;
	uint16_t members;
	uint8_t leaderId;
	uint8_t time;
	uint8_t data[26];
	
      } msg; __attribute__((packed)); 
    };
} ;

typedef struct _tNRFPacket NRFPacket;

void nrfInit();
bool nrfTest();
bool nrfSendPacket(LDMap *ldmap);
bool nrfDeliveryPacket(EsbPacket *data);



#endif
