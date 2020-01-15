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

/* Author: Wilkins White */

#if defined(__USE_CMSIS)

#include "port_handler_cmsis.h"
#include <stdlib.h>
#include <string.h>

#define NAME_SIZE 16
#define LATENCY_TIMER 10

typedef struct
{
  const char *name;
  ARM_DRIVER_USART *driver;
} Device;

static Device deviceLookup[] = {
#if RTE_USART0
 {"USART0", &Driver_USART0 },
#endif

#if RTE_USART1
 {"USART1", &Driver_USART1 },
#endif

#if RTE_USART2
 {"USART2", &Driver_USART2 },
#endif

#if RTE_USART3
 {"USART3", &Driver_USART3 },
#endif

#if RTE_USART4
 {"USART4", &Driver_USART4 },
#endif

#if RTE_USART5
 {"USART5", &Driver_USART5 },
#endif

 { NULL, NULL },
};

typedef struct
{
  ARM_DRIVER_USART *driver;

  int     baudrate;
  char    port_name[NAME_SIZE];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
  uint8_t rx_busy;
} PortData;

static PortData *portData = NULL;

int portHandlerCMSIS(const char *port_name)
{
  int port_num = 0;

  if (portData == NULL)
  {
    g_used_port_num = 1;
    portData = (PortData *)calloc(g_used_port_num, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(g_used_port_num, sizeof(uint8_t));
  }
  else
  {
    // Check for duplicate port initialization
    for (int i = 0; i < g_used_port_num; i++)
    {
      if (strcmp(portData[i].port_name, port_name) == 0)
        return -1;
    }

    // Increase the port count
    port_num = g_used_port_num;

    g_used_port_num += 1;
    portData = (PortData*)realloc(portData, g_used_port_num * sizeof(PortData));
    g_is_using = (uint8_t*)realloc(g_is_using, g_used_port_num * sizeof(uint8_t));
  }

  setPortNameCMSIS(port_num, port_name);

  // Check that the port_name was valid
  if (portData[port_num].driver == NULL)
      return -1;

  g_is_using[port_num] = False;
  return port_num;
}

uint8_t openPortCMSIS(int port_num)
{
  portData[port_num].driver->Initialize(NULL);
  portData[port_num].driver->PowerControl(ARM_POWER_FULL);

  if(portData[port_num].baudrate)
    setBaudRateCMSIS(port_num, portData[port_num].baudrate);

  return 0;
}

void closePortCMSIS(int port_num)
{
  clearPortCMSIS(port_num);

  portData[port_num].driver->Uninitialize();
  portData[port_num].driver->PowerControl(ARM_POWER_OFF);
}

void clearPortCMSIS(int port_num)
{
  if (!portData[port_num].rx_busy)
      return;

  portData[port_num].driver->Control(ARM_USART_ABORT_RECEIVE, 0);
  portData[port_num].rx_busy = False;
}

void setPortNameCMSIS(int port_num, const char *port_name)
{
  int index = 0;
  portData[port_num].driver = NULL;

  while (deviceLookup[index].name != NULL) {
    if (strcmp(deviceLookup[index].name, port_name) == 0)
    {
      strncpy(portData[port_num].port_name, deviceLookup[index].name, NAME_SIZE);
      portData[port_num].driver = deviceLookup[index].driver;
      portData[port_num].rx_busy = False;
      break;
    }
  }
}

char *getPortNameCMSIS(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateCMSIS(int port_num, const int baudrate)
{
  portData[port_num].baudrate = baudrate;
  if (portData[port_num].driver->Control(ARM_USART_MODE_ASYNCHRONOUS, baudrate) < 0)
    return False;

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;
}

int getBaudRateCMSIS(int port_num)
{
  return portData[port_num].baudrate;
}

int readPortCMSIS(int port_num, uint8_t *packet, int length)
{
  if(!packet || length == 0)
      return 0;

  if (!portData[port_num].rx_busy)
  {
    // Start receive
    portData[port_num].driver->Receive(packet, length);
    portData[port_num].rx_busy = True;
    return 0;
  }

  // Wait for transfer to finish
  if(portData[port_num].driver->GetStatus().rx_busy)
    return 0;

  portData[port_num].rx_busy = False;
  return portData[port_num].driver->GetRxCount();
}

int writePortCMSIS(int port_num, uint8_t *packet, int length)
{
  if(!packet || length == 0)
      return 0;

  enableTransmitCMSIS();
  portData[port_num].driver->Send(packet, length);

  // Block for write
  while(portData[port_num].driver->GetStatus().tx_busy) {};
  disableTransmitCMSIS();

  return portData[port_num].driver->GetTxCount();
}

void setPacketTimeoutCMSIS(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeCMSIS();
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0);
}

void setPacketTimeoutMSecCMSIS(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeCMSIS();
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutCMSIS(int port_num)
{
  if (getTimeSinceStartCMSIS(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getTimeSinceStartCMSIS(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeCMSIS() - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeCMSIS();

  return time_since_start;
}

#endif
