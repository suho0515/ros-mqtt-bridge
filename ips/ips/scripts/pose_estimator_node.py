#!/usr/bin/env python
import rospy
import sys
import paho.mqtt.client as mqtt
import geometry_msgs.msg 
import json
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler 
import message_filters
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int16

class POSE_ESTIMATOR:
    def __init__(self):
        self.dual_tag_left = rospy.get_param('/pose_estimator_node/dual_tag_left')
        self.dual_tag_right = rospy.get_param('/pose_estimator_node/dual_tag_right')

        self.dual_tag_left_position = self.dual_tag_left + '/position'
        self.dual_tag_right_position = self.dual_tag_right + '/position'

        self.dual_tag_left_position_sub = message_filters.Subscriber(self.dual_tag_left_position, PoseStamped)
        self.dual_tag_right_position_sub = message_filters.Subscriber(self.dual_tag_right_position, PoseStamped)
        
        ts = message_filters.ApproximateTimeSynchronizer([self.dual_tag_left_position_sub, self.dual_tag_right_position_sub], queue_size=5, slop=0.1)
        ts.registerCallback(self.callback)


        self.dual_tag_left_quality = self.dual_tag_left + '/quality'
        self.dual_tag_right_quality = self.dual_tag_right + '/quality'

        self.dual_tag_left_quality_sub = message_filters.Subscriber(self.dual_tag_left_quality, Int16)
        self.dual_tag_right_quality_sub = message_filters.Subscriber(self.dual_tag_right_quality, Int16)
        
        ts2 = message_filters.ApproximateTimeSynchronizer([self.dual_tag_left_quality_sub, self.dual_tag_right_quality_sub], queue_size=5, slop=0.1, allow_headerless=True)
        ts2.registerCallback(self.qualityCallback)

        rospy.Subscriber(self.dual_tag_left, PoseStamped, self.callback)

    def qualityCallback(self, left_tag_quality, right_tag_quality):
        #print(left_tag_quality)
        #print(right_tag_quality)

        self.calculate_quality(left_tag_quality.data, right_tag_quality.data)

    def callback(self, left_tag_position, right_tag_position):
        #print(left_tag_position)
        #print(right_tag_position)

        self.calculate_pose(left_tag_position, right_tag_position)

    def calculate_pose(self, left_tag_position, right_tag_position):
        # print(left_tag_position.pose.position.x)

        x = 0.0
        y = 0.0
        
        rx = right_tag_position.pose.position.x
        ry = right_tag_position.pose.position.y
        rz = right_tag_position.pose.position.z    
        
        lx = left_tag_position.pose.position.x
        ly = left_tag_position.pose.position.y
        lz = left_tag_position.pose.position.z

        x_diff = rx - lx
        y_diff = ry - ly

        theta = math.degrees(math.atan(x_diff/y_diff))
        #print(theta)
        
        # divide value to 4 sector
        if (rx > lx) and (ry > ly):
            #print("1")
            theta = 90.0 - theta
            x = lx + ((rx-lx)/2.0)
            y = ly + ((ry-ly)/2.0)
        elif (rx < lx) and (ry > ly):
            #print("2")
            theta = 90.0 - theta
            x = rx + ((lx-rx)/2.0)
            y = ly + ((ry-ly)/2.0)
        elif (rx < lx) and (ry < ly):
            #print("3")
            theta = 270.0 - theta
            x = rx + ((lx-rx)/2.0)
            y = ry + ((ly-ry)/2.0)            
        elif (rx > lx) and (ry < ly):
            #print("4")
            theta = 270.0 - theta
            x = lx + ((rx-lx)/2.0)
            y = ry + ((ly-ry)/2.0)

        z = (lz+rz)/2.0

        print(theta)
        #print(x)
        #print(y)

        theta = math.radians(theta)
        #print(theta)
        #print("\n")

        q = quaternion_from_euler(0.0, 0.0, theta)
        #print(q)

        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        self.pose_publisher(p)

    def pose_publisher(self, p):
        
        ps = PoseStamped()

        ps.header.frame_id = 'map'
        ps.header.stamp = rospy.Time.now()

        ps.pose = p

        str_name = "dual_pose"
        p_pub = rospy.Publisher(str_name, PoseStamped, queue_size=10)

        p_pub.publish(ps)

    def calculate_quality(self, left_tag_quality, right_tag_quality):
        quality = 0
        if left_tag_quality > right_tag_quality:
            quality = right_tag_quality
        else:
            quality = left_tag_quality

        #print(quality)
        self.quality_publisher(quality)

    def quality_publisher(self, q):
        
        q_msg = Int16()
        #print(q)
        q_msg.data = q
        
        str_name = "dual_pose_quality"
        q_pub = rospy.Publisher(str_name, Int16, queue_size=10)

        q_pub.publish(q_msg)

def main(args):
  rospy.init_node('pose_estimator', anonymous=True)
  pose_estimator = POSE_ESTIMATOR()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)