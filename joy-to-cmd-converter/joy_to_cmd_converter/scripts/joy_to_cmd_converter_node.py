#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import message_filters
import json
from rospy_message_converter import message_converter, json_message_converter
import rospkg
import codecs
import yaml

class JOY_TO_CMD_CONVERTER:
    def __init__(self):
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        self.file_path = rospack.get_path('joy_to_cmd_converter') + "/json/command.json"
        print(self.file_path)
        with codecs.open(self.file_path, 'r', encoding='utf-8') as file:
            self.cmd = json.load(file)
            # self.cmd = json.dumps(self.cmd, ensure_ascii=False)

            # self.cmd = yaml.safe_load(file)
            #print(json.dumps(self.cmd, ensure_ascii=False))
            #self.cmd = json.dumps(self.cmd, ensure_ascii=False)
            print(type(self.cmd))
            print(self.cmd)
            print(self.cmd["data"])
            print(self.cmd["data"]["manual"])

        self.left_joy_sub = rospy.Subscriber("/left/joy",Joy,self.left_joy_callback)
        self.right_joy_sub = rospy.Subscriber("/right/joy",Joy,self.right_joy_callback)

        self.pub = rospy.Publisher('/command', String, queue_size=10)

    def left_joy_callback(self,data):
        # print(data)
        ddata = self.cmd["data"]
        ddata["manual"] = "on"
        print(self.cmd["data"]["manual"])
        self.cmd_publish(self.cmd)


    def right_joy_callback(self,data):
        # print(data)

        print(data.buttons[0])


        # if(data.buttons[0]):
        #     cmd 

        

        cmd = "test"
        self.cmd_publish(cmd)

    def cmd_publish(self, cmd):
        print(cmd)
        cmd = json.dumps(cmd, ensure_ascii=False)
        print(cmd)
        cmd_msg = json_message_converter.convert_json_to_ros_message('std_msgs/String', cmd)

        # cmd_msg = message_converter.convert_dictionary_to_ros_message('std_msgs/String', cmd)
        # rospy.loginfo(cmd_msg)
        # self.pub.publish(cmd_msg)



def main(args):
    joy_to_cmd_converter = JOY_TO_CMD_CONVERTER()
    rospy.init_node('joy_to_cmd_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)