// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples sync_read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 0, position2: 1000}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SyncSetPosition "{id1: 1, id2: 2, position1: 1000, position2: 0}"
 * $ rosservice call /get_position "{id1: 1, id2: 2}"
 *
 * Author: Jaehyun Shim
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/SyncGetPosition.h"
#include "dynamixel_sdk_examples/SyncSetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_POSITION    116

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

bool syncGetPresentPositionCallback(
  dynamixel_sdk_examples::SyncGetPosition::Request & req,
  dynamixel_sdk_examples::SyncGetPosition::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int32_t pos1 = 0;
  int32_t pos2 = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id1);
  if (dxl_addparam_result != true)
  {
    ROS_ERROR_STREAM("Failed to addparam to groupSyncRead for Dynamixel ID " << req.id1);
    return 0;
  }

  dxl_addparam_result = groupSyncRead.addParam((uint8_t)req.id2);
  if (dxl_addparam_result != true)
  {
    ROS_ERROR_STREAM("Failed to addparam to groupSyncRead for Dynamixel ID " << req.id2);
    return 0;
  }

  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    pos1 = groupSyncRead.getData((uint8_t)req.id1, ADDR_PRESENT_POSITION, 4);
    pos2 = groupSyncRead.getData((uint8_t)req.id2, ADDR_PRESENT_POSITION, 4);
    ROS_INFO("getPosition : [POSITION:%d]", pos1);
    ROS_INFO("getPosition : [POSITION:%d]", pos2);
    res.position1 = pos1;
    res.position2 = pos2;
    groupSyncRead.clearParam();
    return true;
  } else {
    ROS_ERROR_STREAM("Failed to get position! Result: " << dxl_comm_result);
    groupSyncRead.clearParam();
    return false;
  }
}

void syncSetPositionCallback(const dynamixel_sdk_examples::SyncSetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  uint8_t param_goal_pos1[4];
  uint8_t param_goal_pos2[4];

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  uint32_t pos1 = (unsigned int)msg->position1; // Convert int32 -> uint32
  param_goal_pos1[0] = DXL_LOBYTE(DXL_LOWORD(pos1));
  param_goal_pos1[1] = DXL_HIBYTE(DXL_LOWORD(pos1));
  param_goal_pos1[2] = DXL_LOBYTE(DXL_HIWORD(pos1));
  param_goal_pos1[3] = DXL_HIBYTE(DXL_HIWORD(pos1));
  uint32_t pos2 = (unsigned int)msg->position2; // Convert int32 -> uint32
  param_goal_pos2[0] = DXL_LOBYTE(DXL_LOWORD(pos2));
  param_goal_pos2[1] = DXL_HIBYTE(DXL_LOWORD(pos2));
  param_goal_pos2[2] = DXL_LOBYTE(DXL_HIWORD(pos2));
  param_goal_pos2[3] = DXL_HIBYTE(DXL_HIWORD(pos2));

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id1, param_goal_pos1);
  if (dxl_addparam_result != true)
  {
    ROS_ERROR_STREAM( "Failed to addparam to groupSyncWrite for Dynamixel ID " << msg->id1);
  }

  dxl_addparam_result = groupSyncWrite.addParam((uint8_t)msg->id2, param_goal_pos2);
  if (dxl_addparam_result != true)
  {
    ROS_ERROR_STREAM( "Failed to addparam to groupSyncWrite for Dynamixel ID " << msg->id2);
  }

  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id1, msg->position1);
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id2, msg->position2);
  } else {
    ROS_ERROR_STREAM("Failed to set position! Result: " << dxl_comm_result);
  }

  groupSyncWrite.clearParam();
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (portHandler->openPort() == false) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (portHandler->setBaudRate(BAUDRATE) == false) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Failed to enable torque for Dynamixel ID " << DXL1_ID);
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Failed to enable torque for Dynamixel ID " << DXL2_ID);
    return -1;
  }

  ros::init(argc, argv, "sync_read_write_node");
  ros::NodeHandle nh;
  ros::ServiceServer sync_get_position_srv = nh.advertiseService("/sync_get_position", syncGetPresentPositionCallback);
  ros::Subscriber sync_set_position_sub = nh.subscribe("/sync_set_position", 10, syncSetPositionCallback);

  while (ros::ok()) {
    usleep(8 * 1000);

    ros::spin();
  }

  portHandler->closePort();
  return 0;
}
