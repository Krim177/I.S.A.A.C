#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "config.h"
#include "robot.h"
#include "autopilot.h"
#include "pathPlanning.h"
#include "state.h"

/*
void read(char* buffer, unsigned int length);
void seekg(uint16_t pos)
void seekg(uint16_t pos, uint8_t loc)
unsigned int tellg()
*/

void requestCommandFile(uint8_t cmd, uint8_t param)
{
	//ExternalCommand command;
	//command.qrcode = cmd;
	//command.globalState = global;
	//command.localState = local;
	
	//xQueueOverwrite(externalCommandQueue, &command);
}   

class JetsonFileStream{
	public:
		uint16_t pos;
		uint8_t offset;
		unsigned int length;

        JetsonFileStream(){
			this->offset = offset;
			this->pos = pos;
		}
		
		//reads from EEPROM object
		void read(char* buffer, unsigned int length){
			for(uint16_t j = 0, i = pos+offset; i < length; i++, j++){
				//buffer[j] = EEPROM.read(i);
			}
		}
		
		//sets position
		void seekg(uint16_t pos){
			this->pos = pos;
		}
		
		//overloaded seekg function sets position w/ relativity
		void seekg(uint16_t pos, uint8_t loc){
			//loc = ios::ios_base()
			if(loc == 0){
				this->pos = pos;
			}
			if(loc == 1){
				this->pos += pos;
			}
			if(loc == 2){
				this->pos = length - pos;
			}
		}
		
		//gets length property
		unsigned int tellg(){
			return length;
		}
		
	
};