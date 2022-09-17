#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
import rospkg

from manipulator import MANIPULATOR
from mobile_base import MOBILE_BASE

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import json

class MOBILE_MANIPULATOR:
    def __init__(self):
        self.mb = MOBILE_BASE()
        self.mp = MANIPULATOR()

        # get an instance of RosPack with the default search paths
        self.rospack = rospkg.RosPack()
        # list all packages, equivalent to rospack list
        self.rospack.list() 
        # get the file path for rospy_tutorials
        self.path = self.rospack.get_path('mobile_manipulator')

        self.path = self.path + "/db/task_db.json"

        self.task_db = self.load_db(self.path)
        self.task_list = []
        self.resp = None

        rospy.Subscriber('/power_mode',String, self.power_mode_callback)
        rospy.Subscriber('/torque_mode',String, self.torque_mode_callback)
        rospy.Subscriber('/control_mode',String, self.control_mode_callback)
        rospy.Subscriber('/autoset',String, self.autoset_callback)
        rospy.Subscriber('/auto',String, self.auto_callback)
        rospy.Subscriber('/manual',Float64MultiArray, self.manual_callback)

    def power_mode_callback(self, power_mode):
        #print(power_mode)
        self.mp.set_power(power_mode.data)
        pass

    def torque_mode_callback(self, torque_mode):
        #print(torque_mode)
        self.mp.set_torque(torque_mode.data)
        pass    

    def control_mode_callback(self, control_mode):
        #print(control_mode)
        self.stop()
        self.set_control_mode(control_mode.data)
        pass

    def autoset_callback(self, autoset):
        if autoset.data == "AUTOSET1":
            self.add_tasks_to_list(self.task_db['task_list']['test_01'])    
        rospy.loginfo("tasks are added to task list. task_list is %s", self.task_list)
        pass

    def auto_callback(self, auto):
        # print(auto)
        self.proc_task_list(auto.data)
        pass

    def set_control_mode(self, control_mode):
        self.control_mode = control_mode
        pass

    def manual_callback(self, msg):
        self.manual_control(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8])
        pass

    def manual_control(self, trigger, mb_trans, mb_orient, mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6):
        if (self.control_mode == 'MANUAL'):
            if trigger:
                self.mb.velocity_control(mb_trans, mb_orient)
            else:
                self.mp.velocity_control(mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6)
        pass

    def stop(self, ):
        self.mb.velocity_control(0.0, 0.0)
        self.mp.velocity_control(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        pass

    def load_db(self, path):
        with open(path, "r") as st_json:
            db = json.load(st_json)
            return db

    def add_tasks_to_list(self, task_list):
        for i in range(len(task_list)):
            self.task_list.append(task_list[i])        

    def proc_task_list(self, auto):
        for i in range(len(self.task_list)):
            if auto == 'PLAY':
                # print(self.task_list[i])
                key, val = self.task_list[0].items()[0]
                # print(key)
                # print(val)
                # print(self.task_db[key][val])
                if key == "mp":
                    self.mp.value_control(self.task_db[key][val])
                    del self.task_list[0]
                elif key == "mb":
                    self.mb.value_control(self.task_db[key][val])
                    del self.task_list[0]
            else:
                break
        pass

