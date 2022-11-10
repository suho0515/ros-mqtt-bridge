#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import paho.mqtt.client as mqtt
import geometry_msgs.msg 
import json
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler 
import message_filters
import math
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int16, Float64
from visualization_msgs.msg import Marker
import numpy as np
from ips.msg import Tag

class POSE_ESTIMATOR:
    def __init__(self):

        self.p = Pose()
        self.q = Quaternion()
        self.quality = 0
        self.a = Float64()
        self.offset = 0.785398
        self.diagonal_length = math.sqrt(math.pow(0.74,2)+math.pow(0.74,2))
        self.node_dict_list = [None, None, None, None]

        self.tag_1 = rospy.get_param('/pose_estimator_node/tag_1')
        self.tag_2 = rospy.get_param('/pose_estimator_node/tag_2')
        self.tag_3 = rospy.get_param('/pose_estimator_node/tag_3')
        self.tag_4 = rospy.get_param('/pose_estimator_node/tag_4')

        self.tag_1_topic = self.tag_1 + '/position'
        self.tag_2_topic = self.tag_2 + '/position'
        self.tag_3_topic = self.tag_3 + '/position'
        self.tag_4_topic = self.tag_4 + '/position'

        rospy.Subscriber(self.tag_1_topic, Tag, self.tag_1_callback)
        rospy.Subscriber(self.tag_2_topic, Tag, self.tag_2_callback)
        rospy.Subscriber(self.tag_3_topic, Tag, self.tag_3_callback)
        rospy.Subscriber(self.tag_4_topic, Tag, self.tag_4_callback)

        rospy.Timer(rospy.Duration(0.1), self.timer)

    def tag_1_callback(self, tag_1):
        self.node_dict_list[0] = tag_1

    def tag_2_callback(self, tag_2):
        self.node_dict_list[1] = tag_2
    
    def tag_3_callback(self, tag_3):
        self.node_dict_list[2] = tag_3

    def tag_4_callback(self, tag_4):
        self.node_dict_list[3] = tag_4

    def timer(self, event):
        node_dict_list = self.node_dict_list

        if node_dict_list == None:
            return

        tag_num = 0
        for i in range(4):
            if node_dict_list[i] != None:
                tag_num = tag_num + 1
        
        if tag_num < 2: 
            return

        if tag_num == 2:
            print(node_dict_list)
            p, theta = self.calculate_dual_pose(node_dict_list)  
            self.recalc_dual_pose(p, theta, self.diagonal_length) 
        elif tag_num == 3:
            p, theta, p1, p2, p3 = self.calculate_triple_pose(node_dict_list)    
            self.recalc_triple_pose(p, theta, p1, p2, p3, self.diagonal_length) 
        elif tag_num == 4:
            p, theta, p1, p2, p3, p4 = self.calculate_quad_pose(node_dict_list) 
            self.recalc_quad_pose(p, theta, p1, p2, p3, p4, self.diagonal_length)    

        # self.calculate_pose(self.node_dict_list)

        self.reset()


    def reset(self):
        self.node_dict_list = [None, None, None, None]

    def calculate_dual_pose(self, node_dict_list):
        x = 0.0
        y = 0.0
        z = 0.0
        
        rx = 0.0
        ry = 0.0
        rz = 0.0

        lx = 0.0
        ly = 0.0
        lz = 0.0

        flag = False

        if ((node_dict_list[0] != None) and (node_dict_list[2] != None)):
            rx = node_dict_list[2].position.x
            ry = node_dict_list[2].position.y
            rz = node_dict_list[2].position.z
            
            lx = node_dict_list[0].position.x
            ly = node_dict_list[0].position.y
            lz = node_dict_list[0].position.z

            flag = True
        elif (node_dict_list[1] != None) and (node_dict_list[3] != None):
            rx = node_dict_list[3].position.x
            ry = node_dict_list[3].position.y
            rz = node_dict_list[3].position.z
            
            lx = node_dict_list[1].position.x
            ly = node_dict_list[1].position.y
            lz = node_dict_list[1].position.z

            flag = False
        else:
            return

        x_diff = rx - lx
        y_diff = ry - ly

        theta = math.degrees(math.atan(x_diff/y_diff))
        
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

        # print(theta)

        theta_offset = 0.0
        if flag:
            theta_offset = theta + 45.0
        else:
            theta_offset = theta - 45.0

        # print(theta)

        self.a = theta_offset

        self.azimuth_publisher(self.a)
        # print(theta)

        ## calculate rotation in here
        theta_offset = math.radians(theta_offset)
        # print(theta)

        q = quaternion_from_euler(0.0, 0.0, theta_offset)
        
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation.x = q[0]
        self.p.orientation.y = q[1]
        self.p.orientation.z = q[2]
        self.p.orientation.w = q[3]

        

        self.pose_publisher("estimated_pose", self.p)
        self.pose_text_marker_publisher(self.p,self.quality,self.a)

        return self.p, theta

    def recalc_dual_pose(self, p, theta, l):
        new_p = p

        # calculate first and second marker position
        x = p.position.x
        y = p.position.y
        z = p.position.z
        # hl: half length
        hl = l/2
        theta = math.radians(theta)

        # cos(theta) = x/hl
        # -> x = hl * cos(theta)
        x_1 = x - hl*math.cos(theta)

        # sin(theta) = y/hl
        # -> y = hl * sin(theta)
        y_1 = y - hl*math.sin(theta)

        z_1 = z

        self.sphere_marker_publisher("dual_tag_1", x_1, y_1, z_1)

        x_2 = x + hl*math.cos(theta)

        # sin(theta) = y/hl
        # -> y = hl * sin(theta)
        y_2 = y + hl*math.sin(theta)

        z_2 = z

        self.sphere_marker_publisher("dual_tag_2", x_2, y_2, z_2)

        self.line_marker_publisher("dual", x_1, y_1, z_2, x_2, y_2, z_2)

    def calculate_triple_pose(self, node_dict_list):
        x = 0.0
        y = 0.0
        z = 0.0
        
        rx = 0.0
        ry = 0.0
        rz = 0.0

        lx = 0.0
        ly = 0.0
        lz = 0.0

        tx = 0.0
        ty = 0.0
        tz = 0.0

        flag = False

        if ((node_dict_list[0] != None) and (node_dict_list[2] != None)):
            rx = node_dict_list[2].position.x
            ry = node_dict_list[2].position.y
            rz = node_dict_list[2].position.z
            # print(node_dict_list[2].position)
            
            lx = node_dict_list[0].position.x
            ly = node_dict_list[0].position.y
            lz = node_dict_list[0].position.z  
            # print("node_dict_list[0].position:\n")
            # print(node_dict_list[0].position)

            if node_dict_list[1] != None:
                tx = node_dict_list[1].position.x
                ty = node_dict_list[1].position.y
                tz = node_dict_list[1].position.z
                
            else:
                tx = node_dict_list[3].position.x
                ty = node_dict_list[3].position.y
                tz = node_dict_list[3].position.z

            flag = True

        elif (node_dict_list[1] != None) and (node_dict_list[3] != None):
            rx = node_dict_list[3].position.x
            ry = node_dict_list[3].position.y
            rz = node_dict_list[3].position.z
            
            lx = node_dict_list[1].position.x
            ly = node_dict_list[1].position.y
            lz = node_dict_list[1].position.z

            if node_dict_list[0] != None:
                tx = node_dict_list[0].position.x
                ty = node_dict_list[0].position.y
                tz = node_dict_list[0].position.z
            else:
                tx = node_dict_list[2].position.x
                ty = node_dict_list[2].position.y
                tz = node_dict_list[2].position.z

            flag = False

        else:
            return

        x_diff = rx - lx
        y_diff = ry - ly

        theta = math.degrees(math.atan(x_diff/y_diff))
        
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

        z = (lz+rz+tz)/3.0

        

        theta_offset = 0.0
        if flag:
            theta_offset = theta + 45.0
        else:
            theta_offset = theta - 45.0

        print(theta)
        if theta_offset > 360.0:
            theta_offset = theta_offset - 360.0

        # print(theta)

        self.a = theta_offset

        self.azimuth_publisher(self.a)
        # print(theta)

        ## calculate rotation in here
        theta_offset = math.radians(theta_offset)
        # print(theta)

        p1 = np.array([lx,ly,lz])
        p2 = np.array([rx,ry,rz])
        p3 = np.array([tx,ty,tz])
        # print("p1:\n")
        # print(p1)

        N = np.cross(p2-p1, p3-p1)
        N /= np.linalg.norm(N)

        q = quaternion_from_euler(N[0], N[1], theta_offset)
        
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation.x = q[0]
        self.p.orientation.y = q[1]
        self.p.orientation.z = q[2]
        self.p.orientation.w = q[3]

        

        self.pose_publisher("estimated_pose", self.p)
        self.pose_text_marker_publisher(self.p,self.quality,self.a)

        return self.p, theta, p1, p2, p3

    def recalc_triple_pose(self, p, theta, p1, p2, p3, l):

        # calculate first and second marker position
        x = p.position.x
        y = p.position.y
        z = p.position.z
        # hl: half length
        hl = l/2
        if (theta >= 0.0) and (theta < 90.0):
            pass
        elif (theta >= 90.0) and (theta < 180.0):
            theta = 180.0 - theta
        elif (theta >= 180.0) and (theta < 270.0):
            theta = theta - 180.0
        elif (theta >= 270.0) and (theta < 360.0):
            theta = 360.0 - theta


        # print(theta)
        theta = math.radians(theta)
        # print(theta)

        # tag_1

        # cos(theta) = x/hl
        # -> x = hl * cos(theta)

        # sin(theta) = y/hl
        # -> y = hl * sin(theta)

        if (p1[0] > x) and (p1[1] > y):
            # print("1")
            x_1 = x + hl*math.cos(theta)
            y_1 = y + hl*math.sin(theta)
        elif (p1[0] > x) and (p1[1] < y):
            # print("4")
            x_1 = x + hl*math.cos(theta)
            y_1 = y - hl*math.sin(theta)
        elif (p1[0] < x) and (p1[1] > y):
            # print("2")
            x_1 = x - hl*math.cos(theta)
            y_1 = y + hl*math.sin(theta)
        elif (p1[0] < x) and (p1[1] < y):
            # print("3")
            x_1 = x - hl*math.cos(theta)
            y_1 = y - hl*math.sin(theta)

        z_1 = z
        # print(math.cos(theta))
        # print(math.sin(theta))
        # print("p1:\n")
        # print(p1)
        # print("x_1, y_1:\n")
        # print(x_1, y_1)

        self.sphere_marker_publisher("triple_tag_1", x_1, y_1, z_1)

        # tag_2
        if (p2[0] > x) and (p2[1] > y):
            x_2 = x + hl*math.cos(theta)
            y_2 = y + hl*math.sin(theta)
        elif (p2[0] > x) and (p2[1] < y):
            x_2 = x + hl*math.cos(theta)
            y_2 = y - hl*math.sin(theta)
        elif (p2[0] < x) and (p2[1] > y):
            x_2 = x - hl*math.cos(theta)
            y_2 = y + hl*math.sin(theta)
        elif (p2[0] < x) and (p2[1] < y):
            x_2 = x - hl*math.cos(theta)
            y_2 = y - hl*math.sin(theta)

        z_2 = z

        self.sphere_marker_publisher("triple_tag_2", x_2, y_2, z_2)

        # tag_3
        if (p3[0] > x) and (p3[1] > y):
            # print("1")
            x_3 = x + hl*math.sin(theta)
            y_3 = y + hl*math.cos(theta)
        elif (p3[0] > x) and (p3[1] < y):
            # print("4")
            x_3 = x + hl*math.sin(theta)
            y_3 = y - hl*math.cos(theta)
        elif (p3[0] < x) and (p3[1] > y):
            # print("2")
            x_3 = x - hl*math.sin(theta)
            y_3 = y + hl*math.cos(theta)
        elif (p3[0] < x) and (p3[1] < y):
            # print("3")
            x_3 = x - hl*math.sin(theta)
            y_3 = y - hl*math.cos(theta)
            print(y)
            print(hl*math.sin(theta))
        
        
        
        z_3 = z

        self.sphere_marker_publisher("triple_tag_3", x_3, y_3, z_3)

        self.triangle_marker_publisher(x_1, y_1, z_1,
                                        x_2, y_2, z_2,
                                        x_3, y_3, z_3)

    def calculate_quad_pose(self, node_dict_list):
        x_1 = 0.0
        y_1 = 0.0
        x_2 = 0.0
        y_2 = 0.0

        z = 0.0
        
        rx = 0.0
        ry = 0.0
        rz = 0.0

        lx = 0.0
        ly = 0.0
        lz = 0.0

        tx = 0.0
        ty = 0.0
        tz = 0.0

        qx = 0.0
        qy = 0.0
        qz = 0.0

        lx = node_dict_list[0].position.x
        ly = node_dict_list[0].position.y
        lz = node_dict_list[0].position.z  

        rx = node_dict_list[2].position.x
        ry = node_dict_list[2].position.y
        rz = node_dict_list[2].position.z
        
        tx = node_dict_list[1].position.x
        ty = node_dict_list[1].position.y
        tz = node_dict_list[1].position.z

        qx = node_dict_list[3].position.x
        qy = node_dict_list[3].position.y
        qz = node_dict_list[3].position.z

        # Calculate for theta_1
        x_diff = rx - lx
        y_diff = ry - ly

        theta = math.degrees(math.atan(x_diff/y_diff))
        
        # divide value to 4 sector
        if (rx > lx) and (ry > ly):
            #print("1")
            theta = 90.0 - theta
            x_1 = lx + ((rx-lx)/2.0)
            y_1 = ly + ((ry-ly)/2.0)
        elif (rx < lx) and (ry > ly):
            #print("2")
            theta = 90.0 - theta
            x_1 = rx + ((lx-rx)/2.0)
            y_1 = ly + ((ry-ly)/2.0)
        elif (rx < lx) and (ry < ly):
            #print("3")
            theta = 270.0 - theta
            x_1 = rx + ((lx-rx)/2.0)
            y_1 = ry + ((ly-ry)/2.0)            
        elif (rx > lx) and (ry < ly):
            #print("4")
            theta = 270.0 - theta
            x_1 = lx + ((rx-lx)/2.0)
            y_1 = ry + ((ly-ry)/2.0)

        z = (lz+rz+tz+qz)/4.0

        # print(theta)
        theta_offset_1 = theta
        # theta_offset_1 = theta + 45.0
        # if theta_offset_1 > 360.0:
        #     theta_offset_1 = theta_offset_1 - 360.0


        # Calculate for theta_2
        x_diff = qx - tx
        y_diff = qy - ty

        theta = math.degrees(math.atan(x_diff/y_diff))
        
        # divide value to 4 sector
        if (qx > tx) and (qy > ty):
            #print("1")
            theta = 90.0 - theta
            x_2 = tx + ((qx-tx)/2.0)
            y_2 = ty + ((qy-ty)/2.0)
        elif (qx < tx) and (qy > ty):
            #print("2")
            theta = 90.0 - theta
            x_2 = qx + ((tx-qx)/2.0)
            y_2 = ty + ((qy-ty)/2.0)
        elif (qx < tx) and (qy < ty):
            #print("3")
            theta = 270.0 - theta
            x_2 = qx + ((tx-qx)/2.0)
            y_2 = qy + ((ty-qy)/2.0)            
        elif (qx > tx) and (qy < ty):
            #print("4")
            theta = 270.0 - theta
            x_2 = tx + ((qx-tx)/2.0)
            y_2 = ty + ((ty-qy)/2.0)

        # print(theta)
        # theta_offset_2 = 0.0
        # if theta < 45.0:
        #     theta_offset_2 = 360.0 - (45.0 - theta)
        # else:
        theta_offset_2 = theta
        
        x = (x_1 + x_2) / 2.0
        y = (y_1 + y_2) / 2.0
        z = (lz+rz+tz+qz)/4.0

        # print(theta_offset_1)
        # print(theta_offset_2)

        theta_medium = (theta_offset_1 + theta_offset_2)/2.0

        # print(theta_medium)
        theta_medium = theta_medium - 45.0
        if theta_medium < 0.0:
            theta_medium = 360.0 + theta_medium
        elif theta_medium > 360.0:
            theta_medium = theta_medium - 360.0

        theta_offset = 0.0
        if abs(theta_offset_2 - theta_offset_1) > 180.0:
            if (theta_offset_1 > 270.0) and (theta_offset_2 < 90.0):
                theta_offset_1 = -(360.0 - theta_offset_1)
                theta_offset = (theta_offset_1 + theta_offset_2)/2.0
                if theta_offset < 0.0:
                    theta_offset = 360.0 + theta_offset
            elif (theta_offset_1 < 90.0) and (theta_offset_2 > 270.0):
                theta_offset_2 = -(360.0 - theta_offset_2)
                theta_offset = (theta_offset_1 + theta_offset_2)/2.0
                if theta_offset < 0.0:
                    theta_offset = 360.0 + theta_offset
        else:
            theta_offset = (theta_offset_1 + theta_offset_2)/2.0


        # print(theta)

        self.a = theta_offset

        self.azimuth_publisher(self.a)
        # print(theta)

        ## calculate rotation in here

        theta_offset

        theta_offset = math.radians(theta_offset)
        # print(theta)

        p1 = np.array([lx,ly,lz])
        p2 = np.array([rx,ry,rz])
        p3 = np.array([tx,ty,tz])
        p4 = np.array([qx,qy,qz])

        N = np.cross(p2-p1, p3-p1)
        N /= np.linalg.norm(N)

        q = quaternion_from_euler(N[0], N[1], theta_offset)
        
        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation.x = q[0]
        self.p.orientation.y = q[1]
        self.p.orientation.z = q[2]
        self.p.orientation.w = q[3]

        

        self.pose_publisher("estimated_pose", self.p)
        self.pose_text_marker_publisher(self.p,self.quality,self.a)

        return self.p, theta_medium, p1, p2, p3, p4       

    def recalc_quad_pose(self, p, theta, p1, p2, p3, p4, l):

        # calculate first and second marker position
        x = p.position.x
        y = p.position.y
        z = p.position.z
        # hl: half length
        hl = l/2

        if (theta >= 0.0) and (theta < 90.0):
            pass
        elif (theta >= 90.0) and (theta < 180.0):
            theta = 180.0 - theta
        elif (theta >= 180.0) and (theta < 270.0):
            theta = theta - 180.0
        elif (theta >= 270.0) and (theta < 360.0):
            theta = 360.0 - theta

        theta = math.radians(theta)

        # tag_1

        # cos(theta) = x/hl
        # -> x = hl * cos(theta)

        # sin(theta) = y/hl
        # -> y = hl * sin(theta)

        if (p1[0] > x) and (p1[1] > y):
            x_1 = x + hl*math.cos(theta)
            y_1 = y + hl*math.sin(theta)
        elif (p1[0] > x) and (p1[1] < y):
            x_1 = x + hl*math.cos(theta)
            y_1 = y - hl*math.sin(theta)
        elif (p1[0] < x) and (p1[1] > y):
            x_1 = x - hl*math.cos(theta)
            y_1 = y + hl*math.sin(theta)
        elif (p1[0] < x) and (p1[1] < y):
            x_1 = x - hl*math.cos(theta)
            y_1 = y - hl*math.sin(theta)

        z_1 = z

        self.sphere_marker_publisher("quad_tag_1", x_1, y_1, z_1)

        # tag_2
        if (p2[0] > x) and (p2[1] > y):
            x_2 = x + hl*math.cos(theta)
            y_2 = y + hl*math.sin(theta)
        elif (p2[0] > x) and (p2[1] < y):
            x_2 = x + hl*math.cos(theta)
            y_2 = y - hl*math.sin(theta)
        elif (p2[0] < x) and (p2[1] > y):
            x_2 = x - hl*math.cos(theta)
            y_2 = y + hl*math.sin(theta)
        elif (p2[0] < x) and (p2[1] < y):
            x_2 = x - hl*math.cos(theta)
            y_2 = y - hl*math.sin(theta)

        z_2 = z

        self.sphere_marker_publisher("quad_tag_2", x_2, y_2, z_2)

        # tag_3
        if (p3[0] > x) and (p3[1] > y):
            x_3 = x + hl*math.sin(theta)
            y_3 = y + hl*math.cos(theta)
        elif (p3[0] > x) and (p3[1] < y):
            x_3 = x + hl*math.sin(theta)
            y_3 = y - hl*math.cos(theta)
        elif (p3[0] < x) and (p3[1] > y):
            x_3 = x - hl*math.sin(theta)
            y_3 = y + hl*math.cos(theta)
        elif (p3[0] < x) and (p3[1] < y):
            x_3 = x - hl*math.sin(theta)
            y_3 = y - hl*math.cos(theta)
        
        z_3 = z

        self.sphere_marker_publisher("quad_tag_3", x_3, y_3, z_3)

        # tag_4
        if (p4[0] > x) and (p4[1] > y):
            x_4 = x + hl*math.sin(theta)
            y_4 = y + hl*math.cos(theta)
        elif (p4[0] > x) and (p4[1] < y):
            x_4 = x + hl*math.sin(theta)
            y_4 = y - hl*math.cos(theta)
        elif (p4[0] < x) and (p4[1] > y):
            x_4 = x - hl*math.sin(theta)
            y_4 = y + hl*math.cos(theta)
        elif (p4[0] < x) and (p4[1] < y):
            x_4 = x - hl*math.sin(theta)
            y_4 = y - hl*math.cos(theta)
        
        z_4 = z

        self.sphere_marker_publisher("quad_tag_4", x_4, y_4, z_4)

        self.quad_marker_publisher(p)

    def calculate_pose(self, node_dict_list):
        # print(left_tag_position.pose.position.x)

        x = 0.0
        y = 0.0
        
        rx = 0.0
        ry = 0.0
        rz = 0.0

        lx = 0.0
        ly = 0.0
        lz = 0.0

        tx = 0.0
        ty = 0.0
        tz = 0.0

        tp = None

        tag_num = 0
        for i in range(4):
            if node_dict_list[i] != None:
                tag_num = tag_num + 1

        if tag_num < 2:
            return

        # print(tag_num)
        sum_flag = None
        sum_a = 0
        sum_b = 0
        if tag_num == 4:
            sum_a = node_dict_list[0].quality.data + node_dict_list[2].quality.data
            sum_b = node_dict_list[1].quality.data + node_dict_list[3].quality.data
            if sum_a >= sum_b:
                sum_flag = True
            else:
                sum_flag = False

        if sum_flag == None:
            if (node_dict_list[0] != None) and (node_dict_list[2] != None):
                rx = node_dict_list[2].position.x
                ry = node_dict_list[2].position.y
                rz = node_dict_list[2].position.z
                
                lx = node_dict_list[0].position.x
                ly = node_dict_list[0].position.y
                lz = node_dict_list[0].position.z

                if tag_num == 3:
                    if node_dict_list[1] != None:
                        tx = node_dict_list[1].position.x
                        ty = node_dict_list[1].position.y
                        tz = node_dict_list[1].position.z
                        tp = 1
                    else:
                        tx = node_dict_list[3].position.x
                        ty = node_dict_list[3].position.y
                        tz = node_dict_list[3].position.z
                        tp = 3
                elif tag_num == 4:
                    if node_dict_list[1].quality > node_dict_list[3].quality:
                        tx = node_dict_list[1].position.x
                        ty = node_dict_list[1].position.y
                        tz = node_dict_list[1].position.z
                        tp = 1
                    elif node_dict_list[3].quality > node_dict_list[1].quality:
                        tx = node_dict_list[3].position.x
                        ty = node_dict_list[3].position.y
                        tz = node_dict_list[3].position.z
                        tp = 3

            elif (node_dict_list[1] != None) and (node_dict_list[3] != None):
                rx = node_dict_list[3].position.x
                ry = node_dict_list[3].position.y
                rz = node_dict_list[3].position.z
                
                lx = node_dict_list[1].position.x
                ly = node_dict_list[1].position.y
                lz = node_dict_list[1].position.z

                if tag_num == 3:
                    if node_dict_list[0] != None:
                        tx = node_dict_list[0].position.x
                        ty = node_dict_list[0].position.y
                        tz = node_dict_list[0].position.z
                        tp = 0
                    else:
                        tx = node_dict_list[2].position.x
                        ty = node_dict_list[2].position.y
                        tz = node_dict_list[2].position.z
                        tp = 2
                elif tag_num == 4:
                    if node_dict_list[0].quality > node_dict_list[2].quality:
                        tx = node_dict_list[0].position.x
                        ty = node_dict_list[0].position.y
                        tz = node_dict_list[0].position.z
                        tp = 0
                    elif node_dict_list[2].quality > node_dict_list[0].quality:
                        tx = node_dict_list[2].position.x
                        ty = node_dict_list[2].position.y
                        tz = node_dict_list[2].position.z
                        tp = 2
            else:
                return
        elif sum_flag == True:
            rx = node_dict_list[2].position.x
            ry = node_dict_list[2].position.y
            rz = node_dict_list[2].position.z
            
            lx = node_dict_list[0].position.x
            ly = node_dict_list[0].position.y
            lz = node_dict_list[0].position.z

            if tag_num == 3:
                if node_dict_list[1] != None:
                    tx = node_dict_list[1].position.x
                    ty = node_dict_list[1].position.y
                    tz = node_dict_list[1].position.z
                    tp = 1
                else:
                    tx = node_dict_list[3].position.x
                    ty = node_dict_list[3].position.y
                    tz = node_dict_list[3].position.z
                    tp = 3
            elif tag_num == 4:
                if node_dict_list[1].quality > node_dict_list[3].quality:
                    tx = node_dict_list[1].position.x
                    ty = node_dict_list[1].position.y
                    tz = node_dict_list[1].position.z
                    tp = 1
                elif node_dict_list[3].quality > node_dict_list[1].quality:
                    tx = node_dict_list[3].position.x
                    ty = node_dict_list[3].position.y
                    tz = node_dict_list[3].position.z
                    tp = 3
        elif sum_flag == False:
            rx = node_dict_list[3].position.x
            ry = node_dict_list[3].position.y
            rz = node_dict_list[3].position.z
            
            lx = node_dict_list[1].position.x
            ly = node_dict_list[1].position.y
            lz = node_dict_list[1].position.z

            if tag_num == 3:
                if node_dict_list[0] != None:
                    tx = node_dict_list[0].position.x
                    ty = node_dict_list[0].position.y
                    tz = node_dict_list[0].position.z
                    tp = 0
                else:
                    tx = node_dict_list[2].position.x
                    ty = node_dict_list[2].position.y
                    tz = node_dict_list[2].position.z
                    tp = 2
            elif tag_num == 4:
                if node_dict_list[0].quality > node_dict_list[2].quality:
                    tx = node_dict_list[0].position.x
                    ty = node_dict_list[0].position.y
                    tz = node_dict_list[0].position.z
                    tp = 0
                elif node_dict_list[2].quality > node_dict_list[0].quality:
                    tx = node_dict_list[2].position.x
                    ty = node_dict_list[2].position.y
                    tz = node_dict_list[2].position.z
                    tp = 2

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

        # print(theta)

        ## calculate rotation in here
        theta = math.radians(theta)
        # print(theta)
        #print("\n")

        q = None
        if tag_num == 2:
            q = quaternion_from_euler(0.0, 0.0, theta)
        elif tag_num > 2:
            p1 = np.array([lx,ly,lz])
            p2 = np.array([rx,ry,rz])
            p3 = np.array([tx,ty,tz])

            N = np.cross(p2-p1, p3-p1)
            N /= np.linalg.norm(N)
            # print(N)
            # print(theta)

            self.triangle_marker_publisher(p1, p2, p3)
            if tp == 0:
                q = quaternion_from_euler(N[0], N[1], (N[2]+0.785398))
            elif tp == 1:
                q = quaternion_from_euler(N[0], N[1], (N[2]+0.785398+1.5708))
            elif tp == 2:
                q = quaternion_from_euler(N[0], N[1], (N[2]+0.785398+1.5708))
            elif tp == 3:
                q = quaternion_from_euler(N[0], N[1], (N[2]+0.785398))
            else:
                return
            # print(tp)
            # elif tag_num == 4:
            #     if node_dict_list[1].quality >= node_dict_list[3].quality:
            #         q = quaternion_from_euler(N[0], N[1], (N[2]+0.785398))
            #     else:
            #         q = quaternion_from_euler(N[0], N[1], (N[2]-0.785398))
            # print(q)
            # q = quaternion_from_euler(N[0], N[1], N[2]+0.785398)

        self.p.position.x = x
        self.p.position.y = y
        self.p.position.z = z
        self.p.orientation.x = q[0]
        self.p.orientation.y = q[1]
        self.p.orientation.z = q[2]
        self.p.orientation.w = q[3]

        self.pose_publisher(self.p)
        self.pose_marker_publisher(self.p,self.quality,self.a)

    def pose_publisher(self, name, p):
        
        ps = PoseStamped()

        ps.header.frame_id = 'map'

        ps.header.stamp = rospy.Time.now()

        ps.pose = p

        str_name = name + "/pose"
        p_pub = rospy.Publisher(str_name, PoseStamped, queue_size=10)

        p_pub.publish(ps)

    def azimuth_publisher(self, a):
        
        a_msg = Float64()

        a_msg.data = a
        
        str_name = "dual_pose_azimuth"
        a_pub = rospy.Publisher(str_name, Float64, queue_size=10)

        a_pub.publish(a_msg)

    def pose_text_marker_publisher(self, p, q, a):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.lifetime.secs = 1.0

        marker.header.stamp = rospy.Time.now()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 9
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.7

        # Set the pose of the marker
        marker.pose.position.x = p.position.x + 3.0
        marker.pose.position.y = p.position.y + 3.0
        marker.pose.position.z = p.position.z + 3.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        px = float(p.position.x)
        py = float(p.position.y)
        pz = float(p.position.z)
        ox = float(p.orientation.x)
        oy = float(p.orientation.y)
        oz = float(p.orientation.z)
        ow = float(p.orientation.w)

        str_px = ""
        str_py = ""
        str_pz = ""
        str_ox = ""
        str_oy = ""
        str_oz = ""
        str_ow = ""
        if px >= 0.0:
            str_px = '+' + format(px, '.3f')
        else:
            str_px = format(px, '.3f')

        if py >= 0.0:
            str_py = '+' + format(py, '.3f')
        else:
            str_py = format(py, '.3f')

        if pz >= 0.0:
            str_pz = '+' + format(pz, '.3f')
        else:
            str_pz = format(pz, '.3f')

        if ox >= 0.0:
            if ox == -0.0:
                str_ox = "+0.000"
            else:
                str_ox = format(ox, '.3f')
        else:
            str_ox = format(ox, '.3f')

        if oy >= 0.0:
            if oy == -0.0:
                str_oy = "+0.000"
            else:
                str_oy = format(oy, '.3f')
        else:
            str_oy = format(oy, '.3f')

        if oz >= 0.0:
            str_oz = '+' + format(oz, '.3f')
        else:
            str_oz = format(oz, '.3f')

        if ow >= 0.0:
            str_ow = '+' + format(ow, '.3f')
        else:
            str_ow = format(ow, '.3f')

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
                    + "azimuth: " + str((-round(a,2)+417)%360.0) + '\xb0\n' \
                    # + "(before applying offset)"
        marker.text = str_text

        str_name = "pose_text_marker"
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


    def triangle_marker_publisher(self, x_1, y_1, z_1,
                                        x_2, y_2, z_2,
                                        x_3, y_3, z_3):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.lifetime.secs = 1.0

        marker.header.stamp = rospy.Time.now()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 11
        marker.id = 1

        # Set the scale of the marker
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        p_msg = Point()
        p_list = []

        p_msg.x = x_1
        p_msg.y = y_1
        p_msg.z = z_1
        p_list.append(p_msg)
        
        p_msg.x = x_2
        p_msg.y = y_2
        p_msg.z = z_2
        p_list.append(p_msg)

        p_msg.x = x_3
        p_msg.y = y_3
        p_msg.z = z_3
        p_list.append(p_msg)

        # print(p_list)


        marker.points = [Point(x_1,y_1,z_1),
                        Point(x_2,y_2,z_2),
                        Point(x_3,y_3,z_3)]

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        str_name = "triangle_marker"
        m_pub = rospy.Publisher(str_name, Marker, queue_size=10)

        # print(marker)

        m_pub.publish(marker)

    def quad_marker_publisher(self, p):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.lifetime.secs = 1.0

        marker.header.stamp = rospy.Time.now()
        
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = 1

        # Set the scale of the marker
        marker.scale.x = 0.74
        marker.scale.y = 0.74
        marker.scale.z = 0.02

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.pose = p

        str_name = "quad_marker"
        m_pub = rospy.Publisher(str_name, Marker, queue_size=10)

        # print(marker)

        m_pub.publish(marker)

    def sphere_marker_publisher(self, name, x, y, z):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.lifetime.secs = 1.0

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.header.stamp = rospy.Time.now()

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        str_name = name + "/sphere_marker"
        m_pub = rospy.Publisher(str_name, Marker, queue_size=10)

        m_pub.publish(marker)      

    def line_marker_publisher(self, name, x_1, y_1, z_1, x_2, y_2, z_2):
        
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.lifetime.secs = 1.0

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 4
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.header.stamp = rospy.Time.now()

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Set the pose of the marker
        marker.points.append(Point(x_1, y_1, z_1))
        marker.points.append(Point(x_2, y_2, z_2))

        str_name = name + "/line_marker"
        m_pub = rospy.Publisher(str_name, Marker, queue_size=10)

        m_pub.publish(marker)     


def main(args):
  rospy.init_node('pose_estimator', anonymous=True)
  pose_estimator = POSE_ESTIMATOR()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)