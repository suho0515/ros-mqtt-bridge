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

import os
import numpy as np
import rospy
from std_msgs.msg import Int8
from dynamixel_sdk import *

class MODEE:

    def __init__(self):

        # Control table address
        self.DXL_ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        self.DXL_ADDR_GOAL_CURRENT      = 102
        self.DXL_ADDR_GOAL_VELOCITY      = 104
        self.DXL_ADDR_PROFILE_ACCELERATION      = 108
        self.DXL_ADDR_PROFILE_VELOCITY      = 112
        self.DXL_ADDR_GOAL_POSITION      = 116
        self.DXL_ADDR_PRESENT_CURRENT   = 126
        self.DXL_ADDR_PRESENT_POSITION   = 132
        
        self.RH_ADDR_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
        self.RH_ADDR_GOAL_CURRENT      = 550
        self.RH_ADDR_PROFILE_ACCELERATION      = 556
        self.RH_ADDR_PROFILE_VELOCITY      = 560
        self.RH_ADDR_GOAL_POSITION      = 564
        self.RH_ADDR_PRESENT_CURRENT   = 574
        self.RH_ADDR_PRESENT_POSITION   = 580

        # Operating mode table address
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_VELOCITY_CONTROL_MODE = 1
        self.ADDR_CURRENT_BASED_POSITION_CONTROL_MODE = 5

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


        # Variables
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        ## valid volume of micropipette
        self.valid_vol = -1
        ## difference of center percentage
        self.docp = 0
        ## process steps for modee feedback control
        self.process = 0
        ## raw degree value of vertical motion unit actuator
        self.present_position = 0
        ## raw current value of vertical motion unit actuator
        self.present_current = 0
        ## command which modee will do
        self.cmd = 0
        ## goal volume for feedback control
        self.goal_vol = -1
        ## process steps for modee control
        self.process = 0
        ## process flage for modee control
        self.flag = False
        ## process count for modee control
        self.cnt = 0
        ## pumping flag
        self.pumping_flag = False
        ## pumping count
        self.pumping_cnt = 0
        ## valid volume flag
        self.vol_flag = False
        self.cmd_list = []

        # Subscriber
        ## it should subscribe valid volume.
        # self.vol_sub = rospy.Subscriber("valid_volume",Int8,self.volCallback)
        self.center_diff_percentage_sub = rospy.Subscriber("center_difference_percentage",Int8,self.centerDiffPercentageCallback)


        # Open port
        try:
            self.portHandler.openPort()
            rospy.loginfo("Succeeded to open the port")
        except:
            rospy.loginfo("Failed to open the port")

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            rospy.loginfo("Succeeded to change the baudrate")
        except:
            rospy.loginfo("Failed to change the baudrate")

        # Enable Dynamixel Torque
        # self.set_torque(False)
        # rospy.sleep(1)
        self.set_torque(True)
        # self.releasing()
        self.move_to_origin()


        # Timer
        ## we would use ros timer for processing tasks.
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.02), self.timerCallback)

    def volCallback(self,data):
        """!
        @brief subscribe valid volume
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - the volume should be ranged 0 to 100
        """
        try:
            # input error check
            if(data.data < 0 or data.data > 100):
                raise VolumeError(data.data)

            # do
            if self.vol_flag == False:
                self.valid_vol = data.data
                self.vol_flag = True
            elif self.vol_flag == True:
                if abs(data.data - self.valid_vol) < 10:
                    self.valid_vol = data.data


            rospy.loginfo("data.data: %s", data.data)

        except Exception as e:
            print(e)
        
        pass

    def centerDiffPercentageCallback(self, data):
        """!
        @brief subscribe center different percentage
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - the percentage should be ranged from -50 to 50
        """
        try:
            # input error check
            if(data.data < -60 or data.data > 60):
                raise CenterDiffPercentageError(data.data)

            # do
            self.docp = data.data

        except Exception as e:
            print(e)
        
        pass

    def timerCallback(self, *args):
        # rospy.loginfo("timer is working")
        if self.cmd_list:
            self.process_command(self.cmd_list[0][0], self.cmd_list[0][1])
        # print(self.cmd_list)

    def set_torque(self, enable):
        if enable:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.ADDR_OPERATING_MODE, self.ADDR_CURRENT_BASED_POSITION_CONTROL_MODE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.loginfo("Gripper has been successfully connected")

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.ADDR_OPERATING_MODE, self.ADDR_CURRENT_BASED_POSITION_CONTROL_MODE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.loginfo("Vertical DYNAMIXEL has been successfully connected")

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.ADDR_OPERATING_MODE, self.ADDR_VELOCITY_CONTROL_MODE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                rospy.loginfo("Rotary DYNAMIXEL has been successfully connected")

            rospy.loginfo("Ready to get & set Position.")
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.ROTARY_DXL_ID, self.DXL_ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            
            rospy.loginfo("Torque of modee is disabled.")

    def value_control(self, val):
        # self.cmd = int(val[0])
        # self.goal_vol = int(val[1])
        cmd_list = [int(val[0]), int(val[1])]
        self.cmd_list.append(cmd_list)
        

    def process_command(self, cmd, goal_vol):
        """!
        @brief process command
        @details
        @n  command is listed like below
        @n  0: Waiting, modee will wait for command
        @n  1: Feedback Control, modee will do feedback control with this command
        @n  2: Aspirating, modee will do aspirating with this command
        @n  3: Dispensing, modee will do dispensing with this command
        @n  4: Pumping, modee will do pumping with this command
        @n  5: stop, modee will stop both actuators(vertical motion unit actuator and rotary motion unit actuator) with this command
        @param[in] cmd: command which modee will do
        @param[in] goal_vol: goal volume for feedback control
        
        @note input constraints: 
        @n - command should be ranged from 0 to 5
        @n - goal volume should be ranged from 0 to 100
        """
        try:
            # input error check
            if(cmd < 0 or cmd > 7):
                raise CommandError(cmd)
            if(goal_vol != -1):
                if(goal_vol < 0 or goal_vol > 100):
                    raise VolumeError(goal_vol)

            # do
            result = False

            if cmd == 0:
                return "WATING"

            elif cmd == 1:
                self.gripping()
                return "GRIPPING"

            elif cmd == 2:
                self.releasing()
                return "RELEASING"

            elif cmd == 3:
                data = rospy.wait_for_message("/valid_volume", Int8, 1)
                self.valid_vol = data.data
                rospy.loginfo("self.valid_vol: %s", self.valid_vol)
                self.feedback_control_process(goal_vol, self.valid_vol)
                return "FEEDBACK_CONTROL"
    
            elif cmd == 4:
                self.aspirating()
                return "ASPIRATING"

            elif cmd == 5:
                self.dispensing()
                return "DISPENSING"

            elif cmd == 6:
                self.pumping()
                return "PUMPING"

            elif cmd == 7:
                self.stop()
                return "STOP"

        except Exception as e:
            print(e)
        pass

    def gripping(self):
        
        offset = 100
        goal_cur = 600
        goal_pos = 700
        acc = 40
        vel = 400

        if self.flag == False:
            self.control_current_based_position(self.GRIPPER_DXL_ID, goal_cur, goal_pos, acc, vel)
            self.flag = True

        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PRESENT_CURRENT)
        if((dxl_present_current < (goal_cur + offset)) and (dxl_present_current > (goal_cur - offset))):
            self.flag = False
            if self.cmd_list:
                self.cmd_list.pop(0)

    def releasing(self):
        offset = 100
        goal_cur = 600
        goal_pos = 0
        acc = 40
        vel = 400

        if self.flag == False:
            self.control_current_based_position(self.GRIPPER_DXL_ID, goal_cur, goal_pos, acc, vel)
            self.flag = True

        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PRESENT_CURRENT)
        if((dxl_present_position < goal_pos + offset) and (dxl_present_position > goal_pos - offset)):
            self.flag = False
            if self.cmd_list:
                self.cmd_list.pop(0)
            
    def feedback_control_process(self, goal_vol, valid_vol):
        """!
        @brief feedback control to adjust micropipette volume
        @details feedback control would be processed like bellow
        @n - process 0: move to present volume
        @n - process 1: feedback control with difference of goal volume and present volume.
        @n if the difference is bigger than 10, it will rotated with XX acceleration and XX velocity
        @n if the difference is bigger than 3, it will rotated with XX acceleration and XX velocity
        @n if the difference is less than 3, it will rotated with XX acceleration and XX velocity
        @n - process 2: feedback control to adjust differnce of center percentage less than 10 percent
        
        @param[in] goal_vol: goal volume
        
        @note input constraints: 
        @n - goal volume should be ranged from 0 to 100
        @return result: result of the process
        """
        try:
            # input error check
            if(goal_vol < 0 or goal_vol > 100):
                raise VolumeError(goal_vol)

            # do
            if(self.process == 0):
                result = self.move_to_origin()
                if(result):
                    self.process = 1
                print("process 0", result)
            
            elif(self.process == 1):
                result = self.move_to_present_volume()
                if(result):
                    self.process = 2
                print("process 1", result)
        
            elif(self.process == 2):
                result = self.feedback_control_with_volume(goal_vol, valid_vol)
                if(result):
                    self.process = 3
                    #self.cnt = 0
                print("process 2", result)
                
            elif(self.process == 3):
                result = self.feedback_control_with_docp(self.docp)
                if(result):
                    self.process = 4
                    print("process 3", result)

            elif(self.process == 4):
                #result = self.stop()
                self.stop()
                #if(result):
                self.process = 5
                print("process 4", result)

            elif(self.process == 5):
                result = self.move_to_origin()
                if(result):
                    self.process = 0
                    self.cmd = 0
                    self.cnt = 0
                    self.flag = False
                    if self.cmd_list:
                        self.cmd_list.pop(0)
                print("process 5", result)
                
            return True

        except Exception as e:
            print(e)
            return False
        pass

    def aspirating(self):
        if self.process == 0:
            offset = 3
            goal_cur = 500
            goal_pos = 3000
            acc = 60
            vel = 60
            if(self.flag == False):
                self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                self.flag = True

            dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_CURRENT)
            if((dxl_present_current < (goal_cur + offset)) and (dxl_present_current > (goal_cur - offset))):
                self.flag = False
                self.process = 1
                self.stop()
        elif self.process == 1:


            if(self.cnt > 5):
                offset = 3
                goal_cur = 500
                goal_pos = 1024
                acc = 60
                vel = 60
                if(self.flag == False):
                    self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                    self.flag = True

                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)
                if((dxl_present_position < goal_pos + offset) and (dxl_present_position > goal_pos - offset)):
                    self.cmd = 0
                    self.cnt = 0
                    self.process = 0
                    self.flag = False
                    if self.cmd_list:
                        self.cmd_list.pop(0)
            else:
                self.cnt = self.cnt + 1

    def dispensing(self):
        if self.process == 0:
            offset = 3
            goal_cur = 800
            goal_pos = 3000
            acc = 60
            vel = 60
            if(self.flag == False):
                self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                self.flag = True
            dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_CURRENT)
            if((dxl_present_current < (goal_cur + offset)) and (dxl_present_current > (goal_cur - offset))):
                self.flag = False
                self.process = 1
                self.stop()
        elif self.process == 1:


            if(self.cnt > 5):
                offset = 3
                goal_cur = 800
                goal_pos = 1024
                acc = 60
                vel = 60
                if(self.flag == False):
                    self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                    self.flag = True

                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)
                if((dxl_present_position < goal_pos + offset) and (dxl_present_position > goal_pos - offset)):
                    self.cmd = 0
                    self.cnt = 0
                    self.process = 0
                    self.flag = False
                    if self.cmd_list:
                        self.cmd_list.pop(0)
            else:
                self.cnt = self.cnt + 1

    def pumping(self):

        if(self.process == 0):
            self.stop()
            offset = 3
            goal_cur = 800
            goal_pos = 1024
            acc = 10
            vel = 50

            self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
            self.process = 1

        elif(self.process == 1):
            if(self.pumping_flag == False):
                offset = 50
                goal_cur = 800
                goal_pos = 3000
                acc = 100
                vel = 200
                
                if(self.flag == False):
                    self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                    self.flag = True

                dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_CURRENT)
                if((dxl_present_current < goal_cur + offset) and (dxl_present_current > goal_cur - offset)):
                    self.flag = False
                    self.pumping_flag = True
                    self.cnt = self.cnt + 1
            else:
                offset = 50
                goal_cur = 400
                goal_pos = 1600
                acc = 100
                vel = 200
                if(self.flag == False):
                    self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                    self.flag = True
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)
                if((dxl_present_position < goal_pos + offset) and (dxl_present_position > goal_pos - offset)):
                    self.flag = False
                    self.pumping_flag = False
                    self.cnt = self.cnt + 1

                if(self.cnt > 11):
                    offset = 3
                    goal_cur = 800
                    goal_pos = 1024
                    acc = 10
                    vel = 50

                    self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
                    self.cmd = 0
                    self.cnt = 0
                    self.process = 0
                    self.flag = False
                    if self.cmd_list:
                        self.cmd_list.pop(0)

    def control_current_based_position(self, id, current, position, acc, vel):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        if id == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.RH_ADDR_PROFILE_ACCELERATION, acc)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.RH_ADDR_PROFILE_VELOCITY, vel)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.RH_ADDR_GOAL_CURRENT, current)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.RH_ADDR_GOAL_POSITION, position)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_PROFILE_ACCELERATION, acc)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_PROFILE_VELOCITY, vel)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.DXL_ADDR_GOAL_CURRENT, current)
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_GOAL_POSITION, position)

    def control_rotation(self,id=3, acc=10,vel=10):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_PROFILE_ACCELERATION, acc)
        # dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_PROFILE_VELOCITY, abs(vel))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.DXL_ADDR_GOAL_VELOCITY, vel)

    def move_to_origin(self):
        """!
        @brief this function move pipette contact to origin position
        @details
        @return result
        @n - if the position is 0, return true.
        @n - if the position is not 0, return false.
        """
        
        acc_2 = 0
        vel_2 = 0
        self.control_rotation(self.ROTARY_DXL_ID, acc_2, vel_2)
        
        goal_cur = 500
        #cur_1 = (100 - self.valid_vol) + 100
        goal_pos = 1024
        acc_1 = 4
        vel_1 = 40
        self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc_1, vel_1)

        offset = 10

        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)
        if((dxl_present_position < goal_pos + offset) and (dxl_present_position > goal_pos - offset)):
            return True 
        else:
            return False

    def move_to_present_volume(self):
        """!
        @brief this function move pipette contact to present volume
        @details
        @return result
        @n - if the current is bigger than 10, return true.
        @n - if the position is not bigger than 10, return false.
        """
        height = self.volume_to_height(self.valid_vol)
        degree = self.height_to_degree(height)
        # degree_offset = 55
        # degree = degree + degree_offset
        goal_pos = self.degree_to_position(degree)
        # goal_pos_offset = -1024
        # goal_pos = goal_pos + goal_pos_offset

        # rospy.loginfo("self.valid_vol: %s", self.valid_vol)
        # rospy.loginfo("height: %s", height)
        # rospy.loginfo("degree: %s", degree)
        # rospy.loginfo("goal_pos: %s", goal_pos)

        goal_cur = 200
        acc = 4
        vel = 40
        self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, 3000, acc, vel)

        offset = 50

        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_CURRENT)
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)

        rospy.loginfo("dxl_present_current: %s", dxl_present_current)
        rospy.loginfo("dxl_present_position: %s", dxl_present_position)
        if((dxl_present_current < goal_cur + offset) and (dxl_present_current > goal_cur - offset)):
            return True 
        else:
            return False

    def control_volume(self, id, volume):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """

        #self.set_operating_mode(id,3)

        

    def volume_to_height(self, volume):
        """!
        @brief convert actuator degree to position
        @details position = degree*(4,095/360)
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        # 25.0 : length of cam follower
        # 25mm is highst height
        # -25mm is lowest height
        # height = height-25.0;
        height = int((0.1*float(volume))- 25.0)
        return height

    def height_to_degree(self, height):
        """!
        @brief convert actuator degree to position
        @details position = degree*(4,095/360)
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        # 25.0 : length of cam follower
        # 25mm is highst height
        # -25mm is lowest height
        # height = height-25.0;
        degree = int(180.0 - ((np.arcsin(height/25.0)*180.0)/np.pi))
        return degree

    def degree_to_position(self, degree):
        """!
        @brief convert actuator degree to position
        @details position = degree*(4,095/360)
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        #print(degree*float(4095/360))
        position = degree*int(4095/360)
        return position

    def feedback_control_with_volume(self, goal_vol, valid_vol):
        diff = self.calc_diff(goal_vol, valid_vol)
        rospy.loginfo("self.valid_vol: %s", self.valid_vol)
        rospy.loginfo("valid_vol: %s", valid_vol)
        rospy.loginfo("diff: %s", diff)
        # print(self.cnt)
        if(diff > 0):
            # move up
            print("MOVE_UP")
            if(abs(diff) < 5):
                self.volume_up(5, -10)
            elif(abs(diff) < 20):
                self.volume_up(30, -60)
            else:
                self.volume_up(60, -120)
            return False
        elif(diff < 0):
            # move down
            print("MOVE_DOWN")
            if(abs(diff) < 5):
                self.volume_down(5, 10)
            elif(abs(diff) < 20):
                self.volume_up(30, 60)
            else:
                self.volume_down(60, 120)
            return False
        else:
            self.stop()
            if(self.cnt > 5):
                # stop modee
                print("STOP")
                self.stop()
                self.cnt = 0
                return True
            else:
                self.cnt = self.cnt + 1
                return False

    def calc_diff(self, goal_vol, cur_vol):
        res = goal_vol - cur_vol
        return res

    def volume_up(self,acc=10,vel=-30):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """

        self.control_rotation(self.ROTARY_DXL_ID, acc, vel)
        
        goal_cur = 10
        goal_pos = 1500
        acc = 1
        vel = 1
        
        self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)

    def volume_down(self,acc=10,vel=30):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """

        self.control_rotation(self.ROTARY_DXL_ID, acc, vel)
        
        goal_cur = 200
        goal_pos = 3000
        acc = 1
        vel = 1
        
        self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)

    def feedback_control_with_docp(self, docp):
        if(docp > 10):
            self.volume_down(5, 10)
            self.stop()
            return False
        elif(docp < -10):
            self.volume_up(5, -10)
            self.stop()
            return False
        else:
            self.stop()
            if(self.cnt > 5):
                # stop modee
                print("STOP")
                self.stop()
                self.cnt = 0
                return True
            else:
                self.cnt = self.cnt + 1
                return False

    def stop(self):
        """!
        @brief control position of actuator
        @details 
        
        @param[in] data: ros Int8 message
        
        @note input constraints: 
        @n - none
        @note output constraints: 
        @n - none
        @return none
        """
        acc = 1000
        vel = 0

        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_POSITION)
        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.VERTICAL_DXL_ID, self.DXL_ADDR_PRESENT_CURRENT)
        goal_cur = dxl_present_current
        goal_pos = dxl_present_position
        self.control_current_based_position(self.VERTICAL_DXL_ID, goal_cur, goal_pos, acc, vel)
        
        # dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PRESENT_POSITION)
        # dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.GRIPPER_DXL_ID, self.RH_ADDR_PRESENT_CURRENT)
        # goal_cur = dxl_present_current
        # goal_pos = dxl_present_position
        # self.control_current_based_position(self.GRIPPER_DXL_ID, goal_cur, goal_pos, acc, vel)
        
        self.control_rotation(self.ROTARY_DXL_ID, acc, vel)
    
    

class VolumeError(Exception):
    def __init__(self, vol):
        self.msg = "range of volum should be 0~100. but the volum is " + str(vol) + "."
    def __str__(self):
        return self.msg    

class CenterDiffPercentageError(Exception):
    def __init__(self, cdp):
        self.msg = "range of Center Different Percentage should be -50~50. but the CDP is " + str(cdp) + "."
    def __str__(self):
        return self.msg    

class CommandError(Exception):
    def __init__(self, cmd):
        self.msg = "range of command should be ranged from 0 to 7. but the command is " + str(cmd) + "."
    def __str__(self):
        return self.msg    