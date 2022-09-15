#!/usr/bin/python
#-*- coding: utf-8 -*-
import rospy

import actionlib
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController
from std_srvs.srv import Trigger
import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

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
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]

class MANIPULATOR:
    def __init__(self):
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        
        timeout = rospy.Duration(30)

        self.set_mode_client = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', SetModeAction)
        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',  SwitchController)
        
        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        
        self.joint_vel_msg = Float64MultiArray()
        self.joint_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        
        # Running up the Manipulator (for Real Robot)
        # resp = self.s_connectToDashboardServer()
        
        # self.set_robot_to_mode(RobotMode.POWER_OFF)
        #rospy.sleep(0.5)
        # self.set_robot_to_mode(RobotMode.RUNNING)
        #rospy.sleep(10)

        # (For Real Robot)
        # self.s_loadProgram("/programs/ros.urp")
        # rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        # resp = self.s_playProgram()
        # rospy.sleep(0.5)

        self.switch_on_controller("joint_group_pos_controller")
        rospy.sleep(0.5)

        self.pub = rospy.Publisher('/joint_group_pos_controller/command',
                                   Float64MultiArray, queue_size=10)
        self.joint_vel_msg = Float64MultiArray()
        self.joint_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.r = rospy.Rate(10)

        pass

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
        print(result)


    def set_power(self, val):
        pass

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
        print(self.joint_vel_msg)
        self.pub.publish(self.joint_vel_msg)
        #self.r.sleep()
        pass

    def value_control(self, val):
        pass
