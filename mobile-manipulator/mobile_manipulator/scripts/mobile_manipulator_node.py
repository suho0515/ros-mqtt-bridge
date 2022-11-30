#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from mobile_manipulator import MOBILE_MANIPULATOR

if __name__ == '__main__':
    rospy.init_node('mobile_manipulator_node') 

    try:
        mobile_manipulator = MOBILE_MANIPULATOR()
        rospy.spin()
        
    except rospy.ROSInterruptException: pass