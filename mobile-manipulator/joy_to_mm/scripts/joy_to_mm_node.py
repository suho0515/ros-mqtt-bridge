#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from joy_to_mm import JOY_TO_MM

if __name__ == '__main__':
    rospy.init_node('joy_to_mm_node') 

    try:
        joy_to_mm = JOY_TO_MM()
        rospy.spin()
    except rospy.ROSInterruptException: pass