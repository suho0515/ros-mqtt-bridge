#!/usr/bin/python
#-*- coding: utf-8 -*-

class MANIPULATOR:
    def __init__(self):
        # self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode) = None
        # self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState) = None
        # self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram) = None
        # self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode) = None
        # self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger) = None
        # self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger) = None
        # self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger) = None
        # self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger) = None
        # self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load) = None
        # self.set_mode_client = actionlib.SimpleActionClient('/ur_hardware_interface/set_mode', SetModeAction) = None
        # self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',  SwitchController) = None
        # self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1) = None
        # self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10) = None
        # self.joint_vel_msg = Float64MultiArray() = None
        # self.joint_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] = None
        pass

    def set_power(self, val):
        pass

    def set_torque(self, val):
        pass

    def set_control_mode(self, val):
        pass

    def stop(self, ):
        pass

    def velocity_control(self, v0, v1, v2, v3, v4, v5):
        pass

    def value_control(self, val):
        pass
