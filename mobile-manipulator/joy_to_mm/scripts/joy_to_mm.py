#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

class JOY_TO_MM:
    def __init__(self):
        self.power_mode = 'OFF'
        self.torque_mode = 'OFF'
        self.control_mode = 'WAIT'
        self.auto = 'STOP'

        self.manual_msg = Float64MultiArray()
        self.manual_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.autoset = None
        self.auto_flag = False

        self.power_mode_pub = rospy.Publisher('power_mode', String, queue_size=10)
        self.torque_mode_pub = rospy.Publisher('torque_mode', String, queue_size=10)
        self.control_mode_pub = rospy.Publisher('control_mode', String, queue_size=10)
        self.manual_pub = rospy.Publisher('manual', Float64MultiArray, queue_size=10)
        self.autoset_pub = rospy.Publisher('autoset', String, queue_size=10)
        self.auto_pub = rospy.Publisher('auto', String, queue_size=10)

        rospy.Subscriber('/left/joy',Joy, self.left_joy_callback)
        rospy.Subscriber('/right/joy',Joy, self.right_joy_callback)
        pass

    def left_joy_callback(self, left_data):
        self.proc_left_joy(left_data)
        pass
        

    def right_joy_callback(self, right_data):
        self.proc_right_joy(right_data)
        pass

    def proc_left_joy(self, left_data):
        self.publish_mp_on_mode(left_data.buttons[7])
        self.publish_mp_off_mode(left_data.buttons[9])
        # publish control mode
        #self.publish_power_mode(left_data.buttons[7])
        # publish torque mode
        #self.publish_torque_mode(left_data.buttons[9])
        # publish control mode
        self.publish_control_mode(left_data.buttons[6],
                                left_data.buttons[8],
                                left_data.buttons[10])
        # publish manual mode
        self.publish_manual(left_data.axes[0],
                            left_data.axes[1],
                            left_data.axes[4],
                            None,
                            None,
                            None,
                            None)
        pass

    def proc_right_joy(self, right_data):
        #print(right_data)
        # publish manual mode
        self.publish_manual(None, 
                            None, 
                            None, 
                            right_data.axes[0],
                            right_data.axes[1],
                            right_data.axes[4],
                            right_data.buttons[0])
        
        # publish autoset
        self.publish_autoset(right_data.buttons[7],
                        right_data.buttons[9],
                        right_data.buttons[11])

        # publish auto play/stop
        self.publish_auto_play(right_data.buttons[6])
        self.publish_auto_stop(right_data.buttons[8])
        self.publish_auto_clear(right_data.buttons[10])
        #self.publish_auto(right_data.buttons[10])
        pass

    def publish_mp_on_mode(self, signal):
        if signal:
            self.power_mode_pub.publish('ON')
        pass
    def publish_mp_off_mode(self, signal):
        if signal:
            self.power_mode_pub.publish('OFF')
        pass

    def publish_power_mode(self, power_mode):
        if power_mode:
            if self.power_mode is 'OFF':
                self.power_mode = 'ON'
            else:
                self.power_mode = 'OFF'
            self.power_mode_pub.publish(self.power_mode)
        pass


    def publish_torque_mode(self, torque_mode):
        if torque_mode:
            if self.torque_mode is 'OFF':
                self.torque_mode = 'ON'
            else:
                self.torque_mode = 'OFF'
            self.torque_mode_pub.publish(self.torque_mode)
        pass

    def publish_control_mode(self, wait, manual, auto):
        if (wait == 1) or (manual == 1) or (auto == 1):
            sum = wait + manual + auto
            if sum == 1:
                if wait:
                    self.control_mode = 'WAIT'
                elif manual:
                    self.control_mode = 'MANUAL'
                elif auto:
                    self.control_mode = 'AUTO'
                self.control_mode_pub.publish(self.control_mode)
        pass

    def publish_manual(self, la0, la1, la4, ra0, ra1, ra4, rb0):
        if self.control_mode is 'MANUAL':
            self.update_manual_msg(rb0, ra1, ra0, la0, ra1, la1, ra0, la4, ra4)
            self.manual_pub.publish(self.manual_msg)
        pass

    def publish_autoset(self, 
                    autoset1, 
                    autoset2, 
                    autoset3):
        if self.control_mode is 'AUTO':
            if (autoset1 == 1) \
                or (autoset2 == 1) \
                or (autoset3 == 1):
                sum = autoset1 \
                    + autoset2 \
                    + autoset3
                if sum == 1:
                    if self.auto_flag is False:
                        if autoset1:
                            self.autoset = 'AUTOSET1'
                        elif autoset2:
                            self.autoset = 'AUTOSET2'
                        elif autoset3:
                            self.autoset = 'AUTOSET3'
                        self.autoset_pub.publish(self.autoset)
                        self.auto_flag = True
            else:
                self.auto_flag = False
        pass

    def publish_auto_play(self, signal):
        if signal:
            self.auto_pub.publish('PLAY')
    def publish_auto_stop(self, signal):
        if signal:
            self.auto_pub.publish('STOP')
    def publish_auto_clear(self, signal):
        if signal:
            self.auto_pub.publish('CLEAR')

    def publish_auto(self, signal):
        if self.control_mode is 'AUTO':
            if auto:
                if self.auto is 'STOP':
                    self.auto = 'PLAY'
                else:
                    self.auto = 'STOP'
                self.auto_pub.publish(self.auto)
        pass

    def update_manual_msg(self, trigger, mb_trans, mb_orient, mp_v1, mp_v2, mp_v3, mp_v4, mp_v5, mp_v6):
        if trigger is not None:
            self.manual_msg.data[0] = trigger
        if mb_trans is not None:
            self.manual_msg.data[1] = mb_trans
        if mb_orient is not None:
            self.manual_msg.data[2] = mb_orient
        if mp_v1 is not None:
            self.manual_msg.data[3] = mp_v1
        if mp_v2 is not None:
            self.manual_msg.data[4] = mp_v2
        if mp_v3 is not None:
            self.manual_msg.data[5] = mp_v3
        if mp_v4 is not None:
            self.manual_msg.data[6] = mp_v4
        if mp_v5 is not None:
            self.manual_msg.data[7] = mp_v5
        if mp_v6 is not None:
            self.manual_msg.data[8] = mp_v6
        pass