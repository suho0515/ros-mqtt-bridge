#!/usr/bin/env python

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class ModEE:

    def __init__(self):

        # Control table address
        self.DXL_ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.DXL_ADDR_GOAL_CURRENT      = 102
        self.DXL_ADDR_GOAL_VELOCITY      = 104
        self.DXL_ADDR_PROFILE_ACCELERATION      = 108
        self.DXL_ADDR_PROFILE_VELOCITY      = 112
        self.DXL_ADDR_GOAL_POSITION      = 116
        self.DXL_ADDR_PRESENT_POSITION   = 132
        
        self.RH_ADDR_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
        self.RH_ADDR_PROFILE_ACCELERATION      = 556
        self.RH_ADDR_PROFILE_VELOCITY      = 560
        self.RH_ADDR_GOAL_POSITION      = 564
        self.RH_ADDR_PRESENT_POSITION   = 580

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.GRIPPER_DXL_ID                      = 1                 # Dynamixel ID : 1
        self.VERTICAL_DXL_ID                      = 2                 # Dynamixel ID : 1
        self.ROTARY_DXL_ID                      = 3                 # Dynamixel ID : 1

        self.BAUDRATE                    = 115200             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        rospy.Subscriber('set_position', SetPosition, self.set_goal_pos_callback)
        rospy.Service('get_position', GetPosition, self.get_present_pos)

        # Open port
        try:
            self.portHandler.openPort()
            rospy.loginfo("Succeeded to open the port")
        except:
            rospy.loginfo("Failed to open the port")
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            rospy.loginfo("Succeeded to change the baudrate")
        except:
            rospy.loginfo("Failed to change the baudrate")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PROFILE_VELOCITY, 400)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PROFILE_ACCELERATION, 40)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            quit()
        elif dxl_error != 0:
            rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            quit()
        else:
            rospy.loginfo("Gripper has been successfully connected")
        
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PROFILE_VELOCITY, 20)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PROFILE_ACCELERATION, 2)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            quit()
        elif dxl_error != 0:
            rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            quit()
        else:
            rospy.loginfo("Vertical DYNAMIXEL has been successfully connected")
        
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, 11, 1)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_PROFILE_VELOCITY, 20)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_PROFILE_ACCELERATION, 2)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            quit()
        elif dxl_error != 0:
            rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            quit()
        else:
            rospy.loginfo("Rotary DYNAMIXEL has been successfully connected")
        


        rospy.loginfo("Ready to get & set Position.")



    def set_goal_pos_callback(self, data):
        print("Set Goal Position of ID %s = %s" % (data.id, data.position))
        if data.id == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, data.id, self.RH_ADDR_GOAL_POSITION, data.position)
        elif data.id == 2:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, data.id, self.DXL_ADDR_GOAL_POSITION, data.position)
        elif data.id == 3:
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, data.id, self.DXL_ADDR_GOAL_POSITION, data.position)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, data.id, self.DXL_ADDR_GOAL_VELOCITY, data.position)
            # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, data.id, self.DXL_ADDR_PROFILE_ACCELERATION, data.position)
        rospy.loginfo("dxl_comm_result: %s, dxl_error: %s", dxl_comm_result, dxl_error)

    def get_present_pos(self, req):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, req.id, self.DXL_ADDR_PRESENT_POSITION)
        print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
        return dxl_present_position


def main(args):
  rospy.init_node('modee_driver_node', anonymous=True)
  modee = ModEE()
  try:
    #modee.control_current_based_position_test()
    #modee.control_current_based_position_with_rotation_test()
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)