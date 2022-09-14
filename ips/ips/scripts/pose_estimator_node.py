#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from std_msgs.msg import Int16, Float64
from visualization_msgs.msg import Marker

class POSE_ESTIMATOR:
    def __init__(self):

        self.p = Pose()
        self.q = Quaternion()
        self.quality = 0
        self.a = Float64()

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

        self.a = theta

        self.azimuth_publisher(self.a)

        #print(theta)
        #print(x)
        #print(y)

        theta = math.radians(theta)
        #print(theta)
        #print("\n")

        self.q = quaternion_from_euler(0.0, 0.0, theta)
        #print(q)

        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation.x = self.q[0]
        self.p.orientation.y = self.q[1]
        self.p.orientation.z = self.q[2]
        self.p.orientation.w = self.q[3]

        self.pose_publisher(self.p)
        self.marker_publisher(self.p,self.quality,self.a)

    def pose_publisher(self, p):
        
        ps = PoseStamped()

        ps.header.frame_id = 'map'
        ps.header.stamp = rospy.Time.now()

        ps.pose = p

        str_name = "dual_pose"
        p_pub = rospy.Publisher(str_name, PoseStamped, queue_size=10)

        p_pub.publish(ps)

    def azimuth_publisher(self, a):
        
        a_msg = Float64()

        a_msg.data = a
        
        str_name = "dual_pose_azimuth"
        a_pub = rospy.Publisher(str_name, Float64, queue_size=10)

        a_pub.publish(a_msg)

    def marker_publisher(self, p, q, a):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 9
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.7

        # Set the pose of the marker
        marker.pose.position.x = p.position.x + 5.0
        marker.pose.position.y = p.position.y + 5.0
        marker.pose.position.z = p.position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        px = format(p.position.x, '.3f')
        py = format(p.position.y, '.3f')
        pz = format(p.position.z, '.3f')
        ox = format(p.orientation.x, '.3f')
        oy = format(p.orientation.y, '.3f')
        oz = format(p.orientation.z, '.3f')
        ow = format(p.orientation.w, '.3f')

        str_px = ""
        str_py = ""
        str_pz = ""
        str_ox = ""
        str_oy = ""
        str_oz = ""
        str_ow = ""
        if px >= 0.0:
            str_px = '+' + str(px)
        else:
            str_px = '-' + str(px)

        if py >= 0.0:
            str_py = '+' + str(py)
        else:
            str_py = '-' + str(py)

        if pz >= 0.0:
            str_pz = '+' + str(pz)
        else:
            str_pz = '-' + str(pz)

        if ox >= 0.0:
            str_ox = '+' + str(ox)
        else:
            str_ox = '-' + str(ox)

        if oy >= 0.0:
            str_oy = '+' + str(oy)
        else:
            str_oy = '-' + str(oy)

        if oz >= 0.0:
            str_oz = '+' + str(oz)
        else:
            str_oz = '-' + str(oz)

        if ow >= 0.0:
            str_ow = '+' + str(ow)
        else:
            str_ow = '-' + str(ow)

        # set text
        str_text = "virtual tag pose:\n" \
                    + "position[m]:\n" \
                    + "  x: " + str_px + "\n" \
                    + "  y: " + str_py + "\n" \
                    + "  z: " + str_pz + "\n" \
                    + "quaternion:\n" \
                    + "  x: " + str_ox + "\n" \
                    + "  y: " + str_oy + "\n" \
                    + "  z: " + str_oz + "\n" \
                    + "  w: " + str_ow + "\n" \
                    + "quality: " \
                    + str(q) + "%\n" \
                    + "\n" \
                    + "azimuth: " + str(round(a,2)) + '\xb0\n' \
                    + "(before applying offset)"
        marker.text = str_text

        str_name = "dual_pose_marker"
        m_pub = rospy.Publisher(str_name, Marker, queue_size=10)

        m_pub.publish(marker)

    def calculate_quality(self, left_tag_quality, right_tag_quality):
        quality = 0
        if left_tag_quality > right_tag_quality:
            quality = right_tag_quality
        else:
            quality = left_tag_quality

        #print(quality)
        self.quality = quality
        self.quality_publisher(self.quality)

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