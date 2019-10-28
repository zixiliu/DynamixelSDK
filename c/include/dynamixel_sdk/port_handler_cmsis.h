/** Port handler for a CMSIS USART driver.
 *
 * The user must provide the following:
 *   1. RTE_Device.h - Run-time environment file for your implementation.
 *   2. getCurrentTimeCMSIS() - timing interface.
 *   3. enableTransmitCMSIS() - set hardware for transmit
 *   4. disableTransmitCMSIS() - set hardware for receive
 *
 * @warning CMSIS implementation *must* support buffered receive.
 *
 * This is usually enabled in your "RTE_Device.h" file, for example:
 *   #define USART_RX_BUFFER_LEN 256
 *   #define USART0_RX_BUFFER_ENABLE 1
 *
 * For more information on CMSIS see:
 * https://github.com/ARM-software/CMSIS_5
 *
 * @file port_handler_cmsis.h
 * @author Wilkins White
 */

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_CMSIS_PORTHANDLERCMSIS_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_CMSIS_PORTHANDLERCMSIS_C_H_

#include "port_handler.h"
#include "Driver_USART.h"
#include "RTE_Device.h"

#if !(RTE_USART0 || RTE_USART1 || RTE_USART2 || RTE_USART3 || RTE_USART4 || RTE_USART5)
#error "No USART drivers found"
#endif

#if RTE_USART0
  extern ARM_DRIVER_USART Driver_USART0;
#endif

#if RTE_USART1
  extern ARM_DRIVER_USART Driver_USART1;
#endif

#if RTE_USART2
  extern ARM_DRIVER_USART Driver_USART2;
#endif

#if RTE_USART3
  extern ARM_DRIVER_USART Driver_USART3;
#endif

#if RTE_USART4
  extern ARM_DRIVER_USART Driver_USART4;
#endif

#if RTE_USART5
  extern ARM_DRIVER_USART Driver_USART5;
#endif

extern double getCurrentTimeCMSIS();
extern void enableTransmitCMSIS();
extern void disableTransmitCMSIS();

int portHandlerCMSIS            (const char *port_name);

uint8_t openPortCMSIS           (int port_num);
void    closePortCMSIS          (int port_num);
void    clearPortCMSIS          (int port_num);

void    setPortNameCMSIS        (int port_num, const char *port_name);
char   *getPortNameCMSIS        (int port_num);

uint8_t setBaudRateCMSIS        (int port_num, const int baudrate);
int     getBaudRateCMSIS        (int port_num);

int     getBytesAvailableCMSIS  (int port_num);

int     readPortCMSIS           (int port_num, uint8_t *packet, int length);
int     writePortCMSIS          (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutCMSIS     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecCMSIS (int port_num, double msec);
uint8_t isPacketTimeoutCMSIS      (int port_num);

double getTimeSinceStartCMSIS     (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_CMSIS_PORTHANDLERCMSIS_C_H_ */
