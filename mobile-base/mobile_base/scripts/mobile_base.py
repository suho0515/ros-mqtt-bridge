#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

class MOBILE_BASE:
    def __init__(self):
        self.mb_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mb_vel_msg = Twist()
        self.mb_vel_msg.linear.x = 0.0
        self.mb_vel_msg.linear.y = 0.0
        self.mb_vel_msg.linear.z = 0.0
        self.mb_vel_msg.angular.x = 0.0
        self.mb_vel_msg.angular.y = 0.0
        self.mb_vel_msg.angular.z = 0.0

    def stop(self, ):
        pass

    def velocity_control(self, lx, ly, lz, ax, ay, az):
        pass

    def value_control(self, val):
        pass

