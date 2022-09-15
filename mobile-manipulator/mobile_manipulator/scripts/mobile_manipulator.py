#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy

from manipulator import MANIPULATOR
from mobile_base import MOBILE_BASE

from sensor_msgs.msg import Joy

class MOBILE_MANIPULATOR:
    def __init__(self):
        self.mb = MOBILE_BASE()
        self.mp = MANIPULATOR()

        self.power_mode = None
        self.torque_mode = None
        self.control_mode = 'WAIT'
        self.task_list = None
        self.resp = None

        rospy.Subscriber('/left/joy',Joy, self.left_joy_callback)
        rospy.Subscriber('/right/joy',Joy, self.right_joy_callback)

    def left_joy_callback(self, left_data):
        #print(left_data)
        self.proc_left_joy(left_data)
        # self.joint_vel_msg.data[0] = left_data.axes[0]
        # self.joint_vel_msg.data[2] = -left_data.axes[1]
        # self.joint_vel_msg.data[4] = (left_data.axes[4]*0.1)
        pass
        

    def right_joy_callback(self, right_data):
        #print(right_data)
        self.proc_right_joy(right_data)
        # self.joint_vel_msg.data[3] = right_data.axes[0]
        # self.joint_vel_msg.data[1] = right_data.axes[1]
        # self.joint_vel_msg.data[5] = (right_data.axes[4]*0.1)
        pass

    def proc_left_joy(self, left_data):
        #print(left_data)
        if (left_data.buttons[6] == 1) \
            or (left_data.buttons[8] == 1) \
            or (left_data.buttons[10] == 1):
            sum = left_data.buttons[6] \
                + left_data.buttons[8] \
                + left_data.buttons[10]
            if sum == 1:
                if left_data.buttons[6]:
                    self.change_control_mode('WAIT')
                    rospy.loginfo("Control mode is changed to 'WAIT'")
                elif left_data.buttons[8]:
                    self.change_control_mode('MANUAL')
                    rospy.loginfo("Control mode is changed to 'MANUAL'")
                elif left_data.buttons[10]:
                    self.change_control_mode('AUTO')
                    rospy.loginfo("Control mode is changed to 'AUTO'")

        self.manual_control(left_data.axes[0],
                            left_data.axes[1],
                            left_data.axes[4],
                            None,
                            None,
                            None,
                            None,)
        pass

    def proc_right_joy(self, right_data):
        #print(right_data)
        self.manual_control(None, 
                            None, 
                            None, 
                            right_data.buttons[0],
                            right_data.axes[0],
                            right_data.axes[1],
                            right_data.axes[4])
        pass

    def power_mode_sub(self, msg):
        pass

    def change_power_mode(self, power_mode):
        pass

    def torque_mode_sub(self, msg):
        pass

    def change_torque_mode(self, torque_mode):
        pass

    def change_control_mode(self, control_mode):
        self.control_mode = control_mode
        #print(self.control_mode)
        pass

    def manual_control(self, la0=None, la1=None, la4=None, rb0=None, ra0=None, ra1=None, ra4=None):
        if (self.control_mode == 'MANUAL'):
            if rb0:
                self.mb.velocity_control(translation, orientation)
            else:
                self.mp.velocity_control(la0, ra1, la1, ra0, la4, ra4)
            
        pass

    def auto_sub(self, msg):
        pass

    def stop(self, ):
        pass

    def add_tasks_to_list(self, task_list):
        pass