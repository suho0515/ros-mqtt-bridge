#!/usr/bin/env python
import rospy
import sys
import paho.mqtt.client as mqtt
import geometry_msgs.msg 
import json
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler 

class IPS:
    def __init__(self):
        self.ip_address = rospy.get_param('/ips_node/ip_address')
        self.port = rospy.get_param('/ips_node/port')

        self.node_dict_list = []
        self.p_list = []

        self.client = mqtt.Client()
    
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message

        self.client.connect(self.ip_address, self.port)
        # self.client.subscribe('#', 1)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            # to reconnect to mqtt broker even if the connection is broken.
            # run the subscriber in on_connect handler
            self.client.subscribe('#', 1)
            print("connected OK")
        else:
            print("Bad connection Returned code=", rc)


    def on_disconnect(self, client, userdata, flags, rc=0):
        print(str(rc))


    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("subscribed: " + str(mid) + " " + str(granted_qos))


    def on_message(self, client, userdata, msg):
        #print(str(msg.topic.decode("utf-8")))
        #print(str(msg.payload.decode("utf-8")))
        self.message_processor(str(msg.topic.decode("utf-8")), msg.payload)

    def message_processor(self, topic, payload):
        if 'config' in topic:
            self.node_dict_list.append(self.config_msg_to_dict(topic, payload))
        elif 'status' in topic:
            self.node_dict_list = self.status_msg_to_dict(self.node_dict_list, topic, payload)
        elif 'location' in topic:
            self.node_dict_list = self.location_msg_to_dict(self.node_dict_list, topic, payload)

        self.position_publisher(self.node_dict_list)
        
        # print(self.node_dict_list)
        # print('\n')

    def config_msg_to_dict(self, topic, payload):
        
        node_dict = {}

        topic_list = topic.split('/')
        id = topic_list[2]

        pl_dict = json.loads(payload)
        label = str(pl_dict['configuration']['label'].decode("utf-8"))
        nodeType = str(pl_dict['configuration']['nodeType'].decode("utf-8"))

        if nodeType=="ANCHOR":
            # initiator = str(pl_dict['configuration']['anchor']['initiator'].decode("utf-8"))
            
            x = pl_dict['configuration']['anchor']['position']['x']
            y = pl_dict['configuration']['anchor']['position']['y']
            z = pl_dict['configuration']['anchor']['position']['z']
            quality = pl_dict['configuration']['anchor']['position']['quality']

            p_dict = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'quality': 0}
            p_dict['x'] = x
            p_dict['y'] = y
            p_dict['z'] = z
            p_dict['quality'] = quality

            node_dict = {'id': id, 'label': label, 'nodeType': nodeType, 'position': p_dict}
        else:
            node_dict = {'id': id, 'label': label, 'nodeType': nodeType}

        return node_dict
                
    def status_msg_to_dict(self, node_dict_list, topic, payload):
        
        topic_list = topic.split('/')
        id = topic_list[2]

        pl_dict = json.loads(payload)
        status = pl_dict['present']

        for i in range(len(node_dict_list)):
            if node_dict_list[i]['id']==id:
                node_dict_list[i]['status'] = status

        return node_dict_list

    def location_msg_to_dict(self, node_dict_list, topic, payload):
        
        topic_list = topic.split('/')
        id = topic_list[2]

        pl_dict = json.loads(payload)
        x = pl_dict['position']['x']
        y = pl_dict['position']['y']
        z = pl_dict['position']['z']
        quality = pl_dict['position']['quality']

        for i in range(len(node_dict_list)):
            if node_dict_list[i]['id']==id:
                node_dict_list[i]['position'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'quality': 0}
                node_dict_list[i]['position']['x'] = x
                node_dict_list[i]['position']['y'] = y
                node_dict_list[i]['position']['z'] = z
                node_dict_list[i]['position']['quality'] = quality

        return node_dict_list

    def position_publisher(self, node_dict_list):
        
        p = PoseStamped()

        for i in range(len(node_dict_list)):
            if 'position' in node_dict_list[i]:
                p.header.frame_id = 'map'
                p.header.stamp = rospy.Time.now()

                p.pose.position.x = node_dict_list[i]['position']['x']  
                p.pose.position.y = node_dict_list[i]['position']['y']
                p.pose.position.z = node_dict_list[i]['position']['z']

                q = quaternion_from_euler(0.0, 0.0, 0.0)
                p.pose.orientation = Quaternion(*q)

                str_name = "/dwm/node/" + node_dict_list[i]['id'] + "/position"
                p_pub = rospy.Publisher(str_name, PoseStamped, queue_size=10)

                p_pub.publish(p)

def main(args):
  rospy.init_node('ips', anonymous=True)
  ips = IPS()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)