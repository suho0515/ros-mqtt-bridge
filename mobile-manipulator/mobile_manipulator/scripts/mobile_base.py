#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

class MOBILE_BASE:
    def __init__(self):
        self.mb_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mb_vel_msg = Twist()

    def stop(self, ):
        pass

    def velocity_control(self, t, o):
        #print(t, o)
        self.mb_vel_msg
        self.mb_vel_msg.linear.x = t
        self.mb_vel_msg.angular.z = o
        self.mb_pub.publish(self.mb_vel_msg)

        pass

    def value_control(self, val):
        pass
