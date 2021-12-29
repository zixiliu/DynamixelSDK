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

// *********     Ping Example      *********
//
// This example is tested with MX series, AX series and USB2DYNAMIXEL or U2D2.
// For other DYNAMIXEL series, refer to the e-Manual(emanual.robotis.com) and modify the control table properties.
// Be sure that the ID and baudrate of DYNAMIXEL modules are properly configured.
// DYNAMIXEL can easily be configured with DYNAMIXEL Wizard 2.0
// https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

import java.util.Scanner;

public class Ping
{
  public static void main(String[] args)
  {
    // Protocol version
    int PROTOCOL_VERSION                = 1;                   // See which protocol version is used in the Dynamixel

    // Default setting
    byte DXL_ID                         = 1;                   // DYNAMIXEL ID: 1
    int BAUDRATE                        = 57600;
    String DEVICENAME                   = "/dev/ttyUSB0";      // Check which port is being used on your controller
                                                               // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    int COMM_SUCCESS                    = 0;                   // Communication Success result value
    int COMM_TX_FAIL                    = -1001;               // Communication Tx Failed

    // Instead of getch
    Scanner scanner = new Scanner(System.in);

    // Initialize DYNAMIXEL class for java
    Dynamixel dynamixel = new Dynamixel();

    // Initialize PortHandler Structs
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    int port_num = dynamixel.portHandler(DEVICENAME);

    // Initialize PacketHandler Structs
    dynamixel.packetHandler();

    int dxl_comm_result = COMM_TX_FAIL;                       // Communication result

    byte dxl_error = 0;                                       // DYNAMIXEL error
    int dxl_model_number;                                     // DYNAMIXEL model number

    // Open port
    if (dynamixel.openPort(port_num))
    {
      System.out.println("Succeeded to open the port!");
    }
    else
    {
      System.out.println("Failed to open the port!");
      System.out.println("Press any key to terminate...");
      scanner.nextLine();
      return;
    }

    // Set port baudrate
    if (dynamixel.setBaudRate(port_num, BAUDRATE))
    {
      System.out.println("Succeeded to change the baudrate!");
    }
    else
    {
      System.out.println("Failed to change the baudrate!");
      System.out.println("Press any key to terminate...");
      scanner.nextLine();
      return;
    }

    // Try to ping the Dynamixel
    // Get DYNAMIXEL model number
    dxl_model_number = dynamixel.pingGetModelNum(port_num, PROTOCOL_VERSION, DXL_ID);
    if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      System.out.println(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      System.out.println(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
      System.out.printf("[ID: %d] ping Succeeded. DYNAMIXEL model number : %d\n", DXL_ID, dxl_model_number);
    }

    // Close port
    dynamixel.closePort(port_num);

    return;
  }
}
