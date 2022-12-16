#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
import rospkg

from manipulator import MANIPULATOR
from mobile_base import MOBILE_BASE
from modee import MODEE

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import json
from threading import Thread, Event

class MOBILE_MANIPULATOR:
    def __init__(self):
        self.auto = 'STOP'
        self.control_mode = 'WAIT'

        self.mp_param = rospy.get_param('/mobile_manipulator/mp')
        self.mb_param = rospy.get_param('/mobile_manipulator/mb')
        self.modee_param = rospy.get_param('/mobile_manipulator/modee')

        # print(self.mp_param)
        # print(self.mb_param)
        if self.mp_param == True:
            self.mp = MANIPULATOR()
        if self.mb_param == True:
            self.mb = MOBILE_BASE()
        if self.modee_param == True:
            self.modee = MODEE()
        

        # get an instance of RosPack with the default search paths
        self.rospack = rospkg.RosPack()
        # list all packages, equivalent to rospack list
        self.rospack.list() 
        # get the file path for rospy_tutorials
        self.path = self.rospack.get_path('mobile_manipulator')

        self.path = self.path + "/db/task_db.json"
        
        self.task_db = self.load_db(self.path)
        self.task_list = []
        self.value_list = []
        self.resp = None
        
        rospy.Subscriber('/power_mode',String, self.power_mode_callback)
        rospy.Subscriber('/torque_mode',String, self.torque_mode_callback)
        rospy.Subscriber('/control_mode',String, self.control_mode_callback)
        rospy.Subscriber('/autoset',String, self.autoset_callback)
        rospy.Subscriber('/auto',String, self.auto_callback)
        rospy.Subscriber('/manual',Float64MultiArray, self.manual_callback)

        self.event = Event()

        rospy.loginfo("mobile manipulator is initialized.")

    def power_mode_callback(self, power_mode):
        #print(power_mode)
        if self.mp_param == True:
            self.mp.set_power(power_mode.data)
        pass

    def torque_mode_callback(self, torque_mode):
        #print(torque_mode)
        if self.mp_param == True:
            self.mp.set_torque(torque_mode.data)
        pass    

    def control_mode_callback(self, control_mode):
        rospy.loginfo("control_mode command is : %s", control_mode)
        print(control_mode)
        self.stop()
        self.set_control_mode(control_mode.data)
        pass

    def autoset_callback(self, autoset):
        self.task_db = self.load_db(self.path)
        if autoset.data == "AUTOSET1":
            self.add_task_to_list(self.task_db['task_list']['test_01'])
        elif autoset.data == "AUTOSET2":
            self.add_task_to_list(self.task_db['task_list']['test_02'])
        elif autoset.data == "AUTOSET3":
            self.add_task_to_list(self.task_db['task_list']['test_03'])
        rospy.loginfo("task list are added to value list.")
        # rospy.loginfo("task_list : %s", self.task_db['task_list']['test_01'])
        rospy.loginfo("value_list : %s", self.value_list)
        pass

    def auto_callback(self, auto):
        print(auto)
        self.auto = auto.data
        if self.auto == 'PLAY':
            self.event.set()
            proc_thread = Thread(target=self.proc_task_list)
            proc_thread.start()
        elif self.auto == 'STOP':
            self.event.clear()
        elif self.auto == 'CLEAR':
            self.event.clear()
            del self.value_list[:]
            self.value_list[:] = []
        pass

    def set_control_mode(self, control_mode):
        self.control_mode = control_mode
        if control_mode == 'WAIT':
            if self.mp_param == True:
                self.mp.switch_on_controller("")
                rospy.loginfo("mobile manipulator control mode is 'WAIT'")
        elif control_mode == 'MANUAL':
            if self.mp_param == True:
                self.mp.switch_on_controller("joint_group_vel_controller")
                rospy.loginfo("mobile manipulator control mode is 'MANUAL'")
        elif control_mode == 'AUTO':
            if self.mp_param == True:
                self.mp.switch_on_controller("forward_cartesian_traj_controller")
                rospy.loginfo("mobile manipulator control mode is 'AUTO'")
        pass

    def manual_callback(self, msg):
        rospy.loginfo("manual command is received by operator.")
        self.manual_control(msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8])
        pass

    def manual_control(self, trigger, mb_trans, mb_orient, mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6):
        print(self.control_mode == 'MANUAL')
        if (self.control_mode == 'MANUAL'):
            if trigger:
                if self.mb_param == True:
                    self.mb.velocity_control(mb_trans, mb_orient)                    
            else:
                print(self.mp_param == True)
                if self.mp_param == True:
                    self.mp.velocity_control(mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6)
                    rospy.loginfo("manipulator velocity control command is sent by mobile manipulator.")
        pass

    def stop(self, ):
        if self.control_mode == 'MANUAL':
            if self.mb_param == True:
                self.mb.velocity_control(0.0, 0.0)
            if self.mp_param == True:
                self.mp.velocity_control(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        pass

    def load_db(self, path):
        with open(path, "r") as st_json:
            db = json.load(st_json)
            return db

    def add_task_to_list(self, task_list):
        for i in range(len(task_list)):
            key, task = task_list[i].items()[0]
            if key == "mp":
                for j in self.task_db[key][task]:
                    value_dict = {'key': 'mp', 'val': j}
                    self.value_list.append(value_dict)
            elif key == "mb":
                value_dict = {'key': 'mb', 'val': self.task_db[key][task]}
                self.value_list.append(value_dict)
            elif key == "modee":
                value_dict = {'key': 'modee', 'val': self.task_db[key][task]}
                self.value_list.append(value_dict)

    def proc_task_list(self):
        
        for i in range(len(self.value_list)):
            rospy.loginfo(self.auto)
            # print(self.value_list)
            if self.event.is_set() == True:
                # key, val = self.value_list[0].items()[0]
                key = self.value_list[0]['key']
                val = self.value_list[0]['val']
                # print(key)
                # print(val)
                # print(key == "mp")
                if key == "mp":
                    if self.mp_param == True:
                        self.mp.value_control(val)
                    del self.value_list[0]
                    # print(self.auto)
                elif key == "mb":
                    if self.mb_param == True:
                        self.mb.value_control(val)
                    del self.value_list[0]
                elif key == "modee":
                    if self.modee_param == True:
                        self.modee.value_control(val)
                    del self.value_list[0]
                # rospy.loginfo("value control [%s] is done", self.value_list[0])
            else:
                self.stop()
                return
        pass

