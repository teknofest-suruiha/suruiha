from suruiha_gazebo_plugins.msg import UAVMessage
from geometry_msgs.msg import Pose
import rospy

from datetime import datetime
from hadsafha.uav_map import Map
import json

# MESSAGE TYPES
POSE_MSG = 0
HANDSHAKE = 1
SYNC_MAP = 2


class CommManager:

    def __init__(self, uav_name, uav_controller):
        self.uav_name = uav_name
        self.msg_publisher = rospy.Publisher('comm_request', UAVMessage, queue_size=10)
        rospy.Subscriber('comm_' + self.uav_name, UAVMessage, self.message_received)
        self.uav_controller = uav_controller
        self.teammate_poses = {}
        self.teammate_detected_objects = {}

    def message_received(self, msg):

        data = json.loads(msg.msg)
        msg_type = int(data["type"])

        if msg_type == POSE_MSG:
            pose = Pose()
            pose.position.x = float(data["data"]["pose"]["position"]["x"])
            pose.position.y = float(data["data"]["pose"]["position"]["y"])
            pose.position.z = float(data["data"]["pose"]["position"]["z"])
            pose.orientation.x = float(data["data"]["pose"]["orientation"]["x"])
            pose.orientation.y = float(data["data"]["pose"]["orientation"]["y"])
            pose.orientation.z = float(data["data"]["pose"]["orientation"]["z"])
            pose.orientation.w = float(data["data"]["pose"]["orientation"]["w"])
            self.teammate_poses[msg.sender] = pose

        elif msg_type == HANDSHAKE:
            dt = datetime.strptime(data["data"]["wakeup_time"], '%H:%M:%S.%f').time()
            priority = int(data["data"]["uav_priority"])
            if self.uav_controller.uav_wakeup_time > dt:
                if self.uav_controller.uav_priority <= priority:
                    self.uav_controller.uav_priority = priority + 1

        elif msg_type == SYNC_MAP:
            detected_objects = data["data"]

            self.teammate_detected_objects[msg.sender] = detected_objects
            #print("DETECTED:")
            #for i in detected_objects:
                #print(str(i)+" || ")
                #print(detected_objects[i])
        else:
            print('Unknown message type:' + str(msg_type))

    def publish_pose(self):

        msg = {}
        msg["type"] = POSE_MSG
        msg["data"] = {}
        msg["data"]["pose"] = {}
        pose = self.uav_controller.get_latest_pose()
        msg["data"]["pose"]["position"] = {
            "x": str(pose.position.x),
            "y": str(pose.position.y),
            "z": str(pose.position.z)
        }
        msg["data"]["pose"]["orientation"] = {
            "x": str(pose.orientation.x),
            "y": str(pose.orientation.y),
            "z": str(pose.orientation.z),
            "w": str(pose.orientation.w)
        }
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = str(json.dumps(msg))
        '''
        {"data": {"pose": {"position": {"y": "-464.161271549", "x": "490.026573252", "z": "0.990515558677"}, "orientation": {"y": "0.000306916529897", "x": "-0.00508605499668", "z": "-0.00219593598453", "w": "0.999984607738"}}}, "type": 0}')
        '''
        self.msg_publisher.publish(uav_msg)

    def publish_handshake(self):
        msg = {}
        msg["type"] = HANDSHAKE
        msg["data"] = {}
        msg["data"]["wakeup_time"] = str(self.uav_controller.uav_wakeup_time)
        msg["data"]["uav_priority"] = str(self.uav_controller.uav_priority)
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = str(json.dumps(msg))
        self.msg_publisher.publish(uav_msg)

    def publish_map(self, uav_map):
        msg = {}
        msg["type"] = SYNC_MAP
        msg["data"] = uav_map.get_objects_json_formatted()
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = str(json.dumps(msg))
        self.msg_publisher.publish(uav_msg)

    def get_team_mate_poses(self):
        return self.teammate_poses

    def message_received_old(self, msg):
        msg_data_fields = msg.msg.split(' ')
        msg_type = int(msg_data_fields[0])
        if msg_type == POSE_MSG:
            pose = Pose()
            pose.position.x = float(msg_data_fields[1])
            pose.position.y = float(msg_data_fields[2])
            pose.position.z = float(msg_data_fields[3])
            pose.orientation.x = float(msg_data_fields[4])
            pose.orientation.y = float(msg_data_fields[5])
            pose.orientation.z = float(msg_data_fields[6])
            pose.orientation.w = float(msg_data_fields[7])
            self.teammate_poses[msg.sender] = pose

        elif msg_type == HANDSHAKE:
            mesaj = str(msg_data_fields[1])
            #print("This is sample handshaking from ", msg.sender, " msg:", mesaj)
            dt = datetime.strptime(msg_data_fields[1], '%H:%M:%S.%f').time()
            priority = int(msg_data_fields[2])

            if(self.uav_controller.uav_wakeup_time > dt):
                if self.uav_controller.uav_priority <= priority:
                    self.uav_controller.uav_priority = priority + 1

            #print("PRIORITY:",self.uav_controller.uav_priority)

        elif msg_type == SYNC_MAP:
            json_str = str(msg_data_fields[1])
            #print("datafields",msg_data_fields)
            detected_objects = json.dumps(json_str)
            self.teammate_detected_objects[msg.sender] = detected_objects
            #print("From : \n"+ msg.sender, "\nobjects:\n", detected_objects)

        else:
            print('Unknown message type:' + str(msg_type))

        #print('comm_manager msg received from:' + msg.sender + ' msg:' + msg.msg)
    def publish_handshake_old(self):
        msg_string_list = []
        msg_string_list.append(str(HANDSHAKE) + ' ')
        msg_string_list.append(str(self.uav_controller.uav_wakeup_time) + ' ')
        msg_string_list.append(str(self.uav_controller.uav_priority) + ' ')
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)
    def publish_map_old(self, map):

        msg_string_list = []
        msg_string_list.append(str(SYNC_MAP) + ' ')

        to_json = json.dumps(map.get_objects_json_formatted())
        print("to_json", to_json)
        msg_string_list.append(to_json + ' ')
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)
    def publish_pose_old(self):
        msg_string_list = []
        pose = self.uav_controller.get_latest_pose()
        msg_string_list.append(str(POSE_MSG) + ' ')
        msg_string_list.append(str(pose.position.x) + ' ')
        msg_string_list.append(str(pose.position.y)+ ' ')
        msg_string_list.append(str(pose.position.z)+ ' ')
        msg_string_list.append(str(pose.orientation.x)+ ' ')
        msg_string_list.append(str(pose.orientation.y)+ ' ')
        msg_string_list.append(str(pose.orientation.z)+ ' ')
        msg_string_list.append(str(pose.orientation.w))
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)
        #print('comm_manager pose published by :',self.uav_name,' pose:',pose)
