#!/usr/bin/python
#-*- coding: utf-8 -*-

# rospy for use ros with python
import rospy

# actionlib for Connect with ROS Action Server
import actionlib

# ur dashboard messages to use with ros service or ros action
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode

# ur dashboard service messages to require service to ur controller
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load

# controller manager service messages to choose controller for ur robot
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController, \
                                        LoadControllerRequest, \
                                        LoadController

# cartesian control messages to control ur robot in cartesian coordinate
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, \
                                        FollowCartesianTrajectoryGoal, \
                                        FollowCartesianTrajectoryResult, \
                                        CartesianTrajectoryPoint

# Trigger Module for standard service
from std_srvs.srv import Trigger

# standard messages for various purpose (e.g. String)
from std_msgs.msg import String, Float64MultiArray

# TFMessage module of tf2 messages, tf means transformation
# It used to get information of end-effector pose 
from tf2_msgs.msg import TFMessage

# Rotation module of scipy package
# It used for convert quaternion to euler or euler to quaternion
from scipy.spatial.transform import Rotation

import numpy as np

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

ALL_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "vel_joint_traj_controller",
    "joint_group_pos_controller",
    "joint_group_vel_controller",
    "forward_joint_traj_controller",
    "forward_cartesian_traj_controller",
    "twist_controller",
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
]

class MANIPULATOR:
    def __init__(self):
        # Class Variables Initialization
        # ========================================
        # timeout for wait any server it connect
        timeout = rospy.Duration(30)

        # Joint velocity control message
        self.joint_vel_msg = Float64MultiArray()
        self.joint_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]



        # ROS Service Initialization
        # ========================================
        # Get Current Robot Mode
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        # Get Current Program State
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        # Get Current Loaded Program
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        # Get Current Safety Mode State
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        # Connect to Dashboard Server
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        # Quit from Dashboard Server
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        # Load Program
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        # Play Program
        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        # Stop Program
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)


        
        # ROS Action Initialization
        # ========================================
        # Connect to Set Mode Action Server from Client
        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Load Controller Action Server from Client
        self.load_controllers_client = rospy.ServiceProxy('/controller_manager/load_controller',
                LoadController)
        try:
            self.load_controllers_client.wait_for_service(timeout)
            print("controller load service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller load service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Switch Controller Action Server from Client
        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
            
        # Connect to Cartesian Controller Action Server from Client
        self.cartesian_passthrough_trajectory_client = actionlib.SimpleActionClient(
            '/forward_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        try:            
            self.cartesian_passthrough_trajectory_client.wait_for_server(timeout)
            rospy.loginfo("cartesian trajectory controller is turned on.")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))



        # ROS Publisher & Subscriber Initialization
        # ========================================
        # script publisher for specific purpose
        self.script_pub = rospy.Publisher("/ur_hardware_interface/script_command", String, queue_size=1)
        
        # velocity control message publisher
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)

        # Subscribe TF Information of the End-Effector
        # self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        


        # Running up the Manipulator
        # ========================================
        # Connect to dashboard server
        resp = self.s_connectToDashboardServer()
        rospy.loginfo("try to connect to dashboard server.\n result: %s",resp)

        # Load ros.urp program file
        resp = self.s_loadProgram("/ros.urp")
        rospy.loginfo("try to load program.\n result: %s",resp)

        self.set_power('OFF')
        rospy.sleep(2.0)
        self.set_power('ON')

        # Try to play external control file
        # When play external control, Robot should be turned on and running
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram()
        rospy.loginfo("try to play external control.\n result: %s",resp)
        rospy.sleep(0.5)

        pass

    def set_power(self, power_mode):
        print(power_mode)
        if power_mode == 'OFF':            
            self.set_robot_to_mode(RobotMode.POWER_OFF)
            rospy.loginfo("Manipulator power is off.")
        elif power_mode == 'ON':            
            self.set_robot_to_mode(RobotMode.RUNNING)
            rospy.loginfo("Manipulator power is on.")
            

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success

    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        rospy.loginfo("switch_on_controller result is %s",result)

    def set_torque(self, val):
        pass

    def set_control_mode(self, val):
        pass

    def stop(self, ):
        pass

    def velocity_control(self, v0=None, v1=None, v2=None, v3=None, v4=None, v5=None):
        #print(v0, v1, v2, v3, v4, v5)
        if v0 is not None:
            self.joint_vel_msg.data[0] = v0
        if v1 is not None:
            self.joint_vel_msg.data[1] = v1
        if v2 is not None:
            self.joint_vel_msg.data[2] = v2
        if v3 is not None:
            self.joint_vel_msg.data[3] = v3
        if v4 is not None:
            self.joint_vel_msg.data[4] = v4
        if v5 is not None:
            self.joint_vel_msg.data[5] = v5
        #print(self.joint_vel_msg)
        self.pub.publish(self.joint_vel_msg)
        #self.r.sleep()
        pass

    # Function for Control UR Robot in the Cartesian Coordinate
    def value_control(self, p):     
        # print(p)

        goal = FollowCartesianTrajectoryGoal()

        point = CartesianTrajectoryPoint()
        point.pose.position.x = p[0]
        point.pose.position.y = p[1]
        point.pose.position.z = p[2]

        rot = Rotation.from_euler('xyz', [p[3], p[4], p[5]], degrees=True)
        rot_quat = rot.as_quat()
        #print(rot_quat)

        point.pose.orientation.x = -rot_quat[0]
        point.pose.orientation.y = -rot_quat[1]
        point.pose.orientation.z = -rot_quat[2]
        point.pose.orientation.w = -rot_quat[3]
        #print(point.pose)

        time_from_start = p[6]

        point.time_from_start = rospy.Duration(time_from_start)
        goal.trajectory.points.append(point)
        print(str(p) +' is appended')
    
        goal.goal_time_tolerance = rospy.Duration(0.6)
        
        self.cartesian_passthrough_trajectory_client.send_goal(goal)
        self.cartesian_passthrough_trajectory_client.wait_for_result()
        result = self.cartesian_passthrough_trajectory_client.get_result()

        rospy.loginfo("Received result SUCCESSFUL.\n result: %s",result)

    # Function for Control UR Robot in the Cartesian Coordinate
    def value_list_control(self, pose_list):     
        # print(pose_list)

        for l in pose_list:
            goal = FollowCartesianTrajectoryGoal()

            point = CartesianTrajectoryPoint()
            point.pose.position.x = l[0]
            point.pose.position.y = l[1]
            point.pose.position.z = l[2]

            rot = Rotation.from_euler('xyz', [l[3], l[4], l[5]], degrees=True)
            rot_quat = rot.as_quat()
            #print(rot_quat)

            point.pose.orientation.x = -rot_quat[0]
            point.pose.orientation.y = -rot_quat[1]
            point.pose.orientation.z = -rot_quat[2]
            point.pose.orientation.w = -rot_quat[3]
            #print(point.pose)

            time_from_start = l[6]

            point.time_from_start = rospy.Duration(time_from_start)
            goal.trajectory.points.append(point)
            print(str(l) +' is appended')
        
            goal.goal_time_tolerance = rospy.Duration(0.6)
            
            self.cartesian_passthrough_trajectory_client.send_goal(goal)
            self.cartesian_passthrough_trajectory_client.wait_for_result()
            result = self.cartesian_passthrough_trajectory_client.get_result()

            rospy.loginfo("Received result SUCCESSFUL.\n result: %s",result)

    # Function for Control UR Robot in the Cartesian Coordinate
    def value_list_control(self, pose_list):     
        print(pose_list)

        for l in pose_list:
            goal = FollowCartesianTrajectoryGoal()

            point = CartesianTrajectoryPoint()
            point.pose.position.x = l[0]
            point.pose.position.y = l[1]
            point.pose.position.z = l[2]

            rot = Rotation.from_euler('xyz', [l[3], l[4], l[5]], degrees=True)
            rot_quat = rot.as_quat()
            #print(rot_quat)

            point.pose.orientation.x = -rot_quat[0]
            point.pose.orientation.y = -rot_quat[1]
            point.pose.orientation.z = -rot_quat[2]
            point.pose.orientation.w = -rot_quat[3]
            #print(point.pose)

            time_from_start = l[6]

            point.time_from_start = rospy.Duration(time_from_start)
            goal.trajectory.points.append(point)
            print(str(l) +' is appended')
        
            goal.goal_time_tolerance = rospy.Duration(0.6)
            
            self.cartesian_passthrough_trajectory_client.send_goal(goal)
            self.cartesian_passthrough_trajectory_client.wait_for_result()
            result = self.cartesian_passthrough_trajectory_client.get_result()

            rospy.loginfo("Received result SUCCESSFUL.\n result: %s",result)

    # def tf_callback(self, tf_msg):
        # print(tf_msg.transforms[0].transform)
        # p = tf_msg.transforms[0].transform.translation
        # x = np.array([p.x,p.y,p.z])
        # v = np.linalg.norm(x)
        # # print(v)
        # if v > 0.6:
        #     self.stop()