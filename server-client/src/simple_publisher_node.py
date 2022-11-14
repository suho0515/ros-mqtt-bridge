#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('simple_msg', String, queue_size=10)
rospy.init_node('simple_publisher_node')
r = rospy.Rate(1) # 1hz
while not rospy.is_shutdown():
   pub.publish("hello world")
   rospy.loginfo("hello world")
   r.sleep()