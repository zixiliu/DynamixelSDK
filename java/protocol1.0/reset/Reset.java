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
// *********     Factory Reset Example      *********
//
// This example is tested with MX series, AX series and USB2DYNAMIXEL or U2D2.
// For other DYNAMIXEL series, refer to the e-Manual(emanual.robotis.com) and modify the control table properties.
// Be sure that the ID and baudrate of DYNAMIXEL modules are properly configured.
// DYNAMIXEL can easily be configured with DYNAMIXEL Wizard 2.0
// https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

// WARNING!!!
// This example resets DYNAMIXEL to factory default values such as ID and Baudrate.
// When multiple DYNAMIXEL modules are connected, executing Factory Reset may cause ID collision or lose of connection.
// https://emanual.robotis.com/docs/en/dxl/protocol1/#factory-reset

import java.util.Scanner;

public class Reset
{
  public static void main(String[] args)
  {
    // Control table address
    short ADDR_MX_BAUDRATE              = 4;                   // Control table address is different in DYNAMIXEL model

    // Protocol version
    int PROTOCOL_VERSION                = 1;                   // See which protocol version is used in the Dynamixel

    // Default setting
    byte DXL_ID                         = 1;                   // DYNAMIXEL ID: 1-
    int BAUDRATE                        = 57600;
    String DEVICENAME                   = "/dev/ttyUSB0";      // Check which port is being used on your controller
                                                               // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    int FACTORYRST_DEFAULTBAUDRATE      = 57600;               // DYNAMIXEL baudrate set by factoryreset
    byte NEW_BAUDNUM                    = 1;                   // New baudnum to recover DYNAMIXEL baudrate as it was
    byte OPERATION_MODE                 = 0x00;                // Mode is unavailable in Protocol 1.0 Reset

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

    int dxl_comm_result = COMM_TX_FAIL;                        // Communication result

    byte dxl_error = 0;                                        // DYNAMIXEL error
    byte dxl_baudnum_read;                                     // Read baudnum

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

    // Read present baudrate of the controller
    System.out.printf("Now the controller baudrate is : %d\n", dynamixel.getBaudRate(port_num));

    // Try factoryreset
    System.out.printf("[ID: %d] Try factoryreset : \n", DXL_ID);
    dynamixel.factoryReset(port_num, PROTOCOL_VERSION, DXL_ID, OPERATION_MODE);
    if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      System.out.println("Aborted");
      System.out.println(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      return;
    }
    else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      System.out.println(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }

    // Wait for reset
    System.out.printf("Wait for reset...\n");
    try
    {
      Thread.sleep(2000);
    }
    catch (InterruptedException e)
    {
      System.out.println(e.getMessage());
    }

    System.out.printf("[ID: %d] factoryReset Success!\n", DXL_ID);

    // Set controller baudrate to dxl default baudrate
    if (dynamixel.setBaudRate(port_num, FACTORYRST_DEFAULTBAUDRATE))
    {
      System.out.printf("Succeed to change the controller baudrate to : %d\n", FACTORYRST_DEFAULTBAUDRATE);
    }
    else
    {
      System.out.println("Failed to change the controller baudrate");
      System.out.println("Press any key to terminate...");
      scanner.nextLine();
      return;
    }

    // Read DYNAMIXEL baudnum
    dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_BAUDRATE);
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
      System.out.printf("[ID: %d] DYNAMIXEL baudnum is now : %d\n", DXL_ID, dxl_baudnum_read);
    }

    // Write new baudnum
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_BAUDRATE, NEW_BAUDNUM);
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
      System.out.printf("[ID: %d] Set DYNAMIXEL baudnum to : %d\n", DXL_ID, NEW_BAUDNUM);
    }

    // Set port baudrate to BAUDRATE
    if (dynamixel.setBaudRate(port_num, BAUDRATE))
    {
      System.out.printf("Succeed to change the controller baudrate to : %d\n", BAUDRATE);
    }
    else
    {
      System.out.println("Failed to change the controller baudrate");
      System.out.println("Press any key to terminate...");
      scanner.nextLine();
      return;
    }

    try
    {
      Thread.sleep(200);
    }
    catch (InterruptedException e)
    {
      System.out.println(e.getMessage());
    }

    // Read DYNAMIXEL baudnum
    dxl_baudnum_read = dynamixel.read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_BAUDRATE);
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
      System.out.printf("[ID: %d] DYNAMIXEL Baudnum is now : %d", DXL_ID, dxl_baudnum_read);
    }

    return;
  }
}
