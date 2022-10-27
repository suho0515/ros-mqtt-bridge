#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# Rotation module of scipy package
# It used for convert quaternion to euler or euler to quaternion
from scipy.spatial.transform import Rotation

class MOBILE_BASE:
    def __init__(self):
        self.mb_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mb_vel_msg = Twist()

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        
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

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = val[0]
        goal.target_pose.pose.position.y = val[1]
        goal.target_pose.pose.position.z = val[2]

        rot = Rotation.from_euler('xyz', [val[3], val[4], val[5]], degrees=True)
        rot_quat = rot.as_quat()

        goal.target_pose.pose.orientation.x = -rot_quat[0]
        goal.target_pose.pose.orientation.y = -rot_quat[1]
        goal.target_pose.pose.orientation.z = -rot_quat[2]
        goal.target_pose.pose.orientation.w = -rot_quat[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
        pass
