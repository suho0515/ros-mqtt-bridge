#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy

from manipulator import MANIPULATOR
from mobile_base import MOBILE_BASE

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

class MOBILE_MANIPULATOR:
    def __init__(self):
        self.mb = MOBILE_BASE()
        self.mp = MANIPULATOR()

        self.power_mode = None
        self.torque_mode = None
        self.control_mode = 'WAIT'
        self.task_list = None
        self.resp = None

        rospy.Subscriber('/power_mode',String, self.power_mode_callback)
        rospy.Subscriber('/control_mode',String, self.control_mode_callback)
        rospy.Subscriber('/manual',Float64MultiArray, self.manual_callback)

    def power_mode_callback(self, power_mode):
        #print(power_mode)
        self.mp.set_power(power_mode.data)
        pass

    def control_mode_callback(self, control_mode):
        print(control_mode)
        self.set_control_mode(control_mode.data)
        pass

    def manual_callback(self, msg):
        self.manual_control(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8])
        pass

    def torque_mode_sub(self, msg):
        pass

    def set_control_mode(self, control_mode):
        self.control_mode = control_mode
        pass

    def manual_control(self, trigger, mb_trans, mb_orient, mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6):
        if (self.control_mode == 'MANUAL'):
            if trigger:
                self.mb.velocity_control(mb_trans, mb_orient)
            else:
                self.mp.velocity_control(mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6)
                pass
            
        pass

    def auto_sub(self, msg):
        pass

    def stop(self, ):
        pass

    def add_tasks_to_list(self, task_list):
        pass

