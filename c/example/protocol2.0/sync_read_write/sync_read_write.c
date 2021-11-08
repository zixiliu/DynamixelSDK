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

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available DYNAMIXEL model on this example : All models using Protocol 2.0
// This example is designed for using two DYNAMIXEL PRO 54-200, and an USB2DYNAMIXEL.
// To use another DYNAMIXEL model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
// Be sure that DYNAMIXEL PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
// Use DYNAMIXEL SDK library
#include "dynamixel_sdk.h"

// Uncomment the definition below when running this example with P series
// #define USE_DYNAMIXEL_P_SERIES

// Control table address differs by DYNAMIXEL model
#ifdef USE_DYNAMIXEL_P_SERIES
  #define ADDR_TORQUE_ENABLE  512
  #define ADDR_GOAL_POSITION  564
  #define ADDR_PRESENT_POSITION  580
#else
  #define ADDR_TORQUE_ENABLE  64
  #define ADDR_GOAL_POSITION  116
  #define ADDR_PRESENT_POSITION  132
#endif

// Data Byte Length
#define LEN_GOAL_POSITION  4
#define LEN_PRESENT_POSITION  4

// Protocol version of DYNAMIXEL
#define PROTOCOL_VERSION  2.0

// These configuration must match to the actual DYNAMIXEL configuration.
// DYNAMIXEL #1 ID: 1, DYNAMIXEL #2 ID: 2
#define DXL1_ID  1
#define DXL2_ID  2
// Most DYNAMIXEL has a default baudrate of 57600
#define BAUDRATE  57600
// Check which serial port is assigned to the U2D2
// ex) Windows: "COM*"   Linux: "/dev/ttyUSB*" Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"
// Value for enabling the torque
#define TORQUE_ENABLE  1
// Value for disabling the torque
#define TORQUE_DISABLE  0
// Minimum & Maximum range of Goal Position.
// Invalid value range will be ignored. Refer to the product eManual.
#ifdef USE_DYNAMIXEL_P_SERIES
  #define DXL_MINIMUM_POSITION_VALUE  0
  #define DXL_MAXIMUM_POSITION_VALUE  100000
#else
  #define DXL_MINIMUM_POSITION_VALUE  0
  #define DXL_MAXIMUM_POSITION_VALUE  1023
#endif
// Moving status flag threshold
#define DXL_MOVING_STATUS_THRESHOLD  20

#define ESC_ASCII_VALUE  0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  // Initialize GroupSyncWrite Structs for Goal Position
  int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

  // Initialize GroupSyncRead Structs for Present Position
  int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  int index = 0;
  // Save the communication result
  int dxl_comm_result = COMM_TX_FAIL;
  // Save the AddParam result
  uint8_t dxl_addparam_result = False;
  // Save the GetParam result
  uint8_t dxl_getdata_result = False;
  // Min and Max Goal positions to iterate
  int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };

  uint8_t dxl_error = 0;
  int32_t dxl1_present_position = 0, dxl2_present_position = 0;

  // Open the serial port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to set the baudrate!\n");
  }
  else
  {
    printf("Failed to set the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable DYNAMIXEL#1 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    printf("DYNAMIXEL#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable DYNAMIXEL#2 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    printf("DYNAMIXEL#%d has been successfully connected \n", DXL2_ID);
  }

  // Add parameter storage for DYNAMIXEL#1 present position value
  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL1_ID);
  if (dxl_addparam_result != True)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    return 0;
  }

  // Add parameter storage for DYNAMIXEL#2 present position value
  dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL2_ID);
  if (dxl_addparam_result != True)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    return 0;
  }

  while (1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Add DYNAMIXEL#1 goal position value to the Syncwrite buffer
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL1_ID, dxl_goal_position[index], LEN_GOAL_POSITION);
    if (dxl_addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
      return 0;
    }

    // Add DYNAMIXEL#2 goal position value to the Syncwrite buffer
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL2_ID, dxl_goal_position[index], LEN_GOAL_POSITION);
    if (dxl_addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
      return 0;
    }

    // Assemble the SyncWrite packet and send the goal position
    groupSyncWriteTxPacket(groupwrite_num);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

    // Clear the Syncwrite parameter buffer
    groupSyncWriteClearParam(groupwrite_num);

    do
    {
      // SyncRead present position
      groupSyncReadTxRxPacket(groupread_num);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

      // Check if groupsyncread data of DYNAMIXEL#1 is available
      dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      if (dxl_getdata_result != True)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
        return 0;
      }

      // Check if groupsyncread data of DYNAMIXEL#2 is available
      dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      if (dxl_getdata_result != True)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
        return 0;
      }

      // Get DYNAMIXEL#1 present position value
      dxl1_present_position = groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

      // Get DYNAMIXEL#2 present position value
      dxl2_present_position = groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position);

    } while ((abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // switch goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }

  // Disable DYNAMIXEL#1 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  // Disable DYNAMIXEL#2 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  // Close the serial port
  closePort(port_num);

  return 0;
}
