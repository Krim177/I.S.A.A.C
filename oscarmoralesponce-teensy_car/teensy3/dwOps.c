/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */

// #include "dwOps.h"
// 
// #include <string.h>
// 
// #include <stm32f0xx_hal.h>
// #include "spi.h"
// 
// extern SPI_HandleTypeDef hspi1;
// 
// // #define DEBUG_SPI
// 
// #define DWM_IRQn EXTI0_1_IRQn
// 
// 
// #ifdef ESP8266
// 	// default ESP8266 frequency is 80 Mhz, thus divide by 4 is 20 MHz
// 	const SPISettings _fastSPI = SPISettings(20000000L, MSBFIRST, SPI_MODE0);
// #else
// 	const SPISettings _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
// #endif
// const SPISettings _slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
// const SPISettings* _currentSPI = &_fastSPI;
// 
// 
// static dwDevice_t *dev;
// 
// // Initialize interrupts
// void dwOpsInit(dwDevice_t *device)
// {
//   dev = device;
// 
//   NVIC_EnableIRQ(DWM_IRQn);
// }
// 
// // Aligned buffer of 128bytes
// // This is used as a "scratch" buffer to the SPI transfers
// // The problem is that the Cortex-m0 only supports 2Bytes-aligned memory access
// uint16_t alignedBuffer[64];
// 
// static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
//                                       const void* data, size_t dataLength)
// {
// 	SPI.beginTransaction(*_currentSPI);
// 	digitalWrite(_ss, LOW);
// 	for(i = 0; i < headerLength; i++) {
// 		SPI.transfer(header[i]); // send header
// 	}
// 	for(i = 0; i < dataLength; i++) {
// 		SPI.transfer(data[i]); // write values
// 	}
// 	delayMicroseconds(5);
// 	digitalWrite(_ss, HIGH);
// 	SPI.endTransaction();
// }
// 
// static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
//                                      void* data, size_t dataLength)
// {
//       SPI.beginTransaction(*_currentSPI);
// 	digitalWrite(_ss, LOW);
// 	for(i = 0; i < headerLength; i++) {
// 		SPI.transfer(header[i]); // send header
// 	}
// 	for(i = 0; i < dataLength; i++) {
// 		data[i] = SPI.transfer(JUNK); // read values
// 	}
// 	delayMicroseconds(5);
// 	digitalWrite(_ss, HIGH);
// 	SPI.endTransaction();
// }
// 
// static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
// {
//   if(speed == dwSpiSpeedLow) {
//     MX_SPI1_Init();
//   } else {
//     MX_SPI1_Init_Fast();
//   }
// }
// 
// static void reset(dwDevice_t* dev)
// {
//   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
//   HAL_Delay(2);
//   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
// }
// 
// static void delayms(dwDevice_t* dev, unsigned int delay)
// {
//   HAL_Delay(delay);
// }
// 
// dwOps_t dwOps = {
//   .spiRead = spiRead,
//   .spiWrite = spiWrite,
//   .spiSetSpeed = spiSetSpeed,
//   .delayms = delayms,
//   .reset = reset,
// };
