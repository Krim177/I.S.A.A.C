#ifndef   __UARTCOMMANDS_TASK__
#define   __UARTCOMMANDS_TASK__

#include "datastructures.h"

#define UART_FIRST_BYTE  15
#define UART_SECOND_BYTE  201




void uartCommandInit();
bool uartCommandTest();
bool uartDeliveryData(LDMap *data);
bool uartDeliveryCommand(UARTCommand *command);




#endif
