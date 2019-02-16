from suruiha_gazebo_plugins.msg import UAVMessage
from geometry_msgs.msg import Pose
import rospy

# MESSAGE TYPES
POSE_MSG = 0
TERRORIST_MSG = 1


class CommManager:
    def __init__(self, uav_name, uav_controller, terrorist_detector):
        self.uav_name = uav_name
        self.msg_publisher = rospy.Publisher('comm_request', UAVMessage, queue_size=10)
        rospy.Subscriber('comm_' + self.uav_name, UAVMessage, self.message_received)
        self.uav_controller = uav_controller
        self.terrorist_detector = terrorist_detector
        self.teammate_poses = {}

    def message_received(self, msg):
        msg_data_fields = msg.msg.split(' ')
        message = msg_data_fields[1:]  # bastaki mesaj turu haric gerisini aliyor
        msg_type = int(msg_data_fields[0])
        tempList = []
        tNo = -1

        if msg_type == TERRORIST_MSG:
            if (message != [""] or message != ['']):
                for i in range(0, len(message) - 1, 6):
                    if message[i] != 'x':
                        tNo = int(message[i + 1].split("_")[1])
                        if (tNo < len(tempList)):
                            tempList[tNo] = [message[i], message[i + 1], float(message[i + 2]), float(message[i + 3]),
                                             float(message[i + 4]), float(message[i + 5])]
                        else:
                            for indis in range(len(tempList), tNo):
                                tempList.insert(indis, [])
                            tempList.insert(tNo,
                                            [message[i], message[i + 1], float(message[i + 2]), float(message[i + 3]),
                                             float(message[i + 4]), float(message[i + 5])])
                    else:
                        tempList.append([])

            # print("terroristArray: " + str(self.terrorist_detector.terroristArray))
            # print("tempList: " + str(tempList))
            self.terrorist_detector.terroristArray = self.terrorist_detector.combineLists(
                self.terrorist_detector.terroristArray,
                tempList)
            # print("terroristArray: " + str(self.terrorist_detector.terroristArray))

        elif msg_type == POSE_MSG:
            pose = Pose()
            pose.position.x = float(msg_data_fields[1])
            pose.position.y = float(msg_data_fields[2])
            pose.position.z = float(msg_data_fields[3])
            pose.orientation.x = float(msg_data_fields[4])
            pose.orientation.y = float(msg_data_fields[5])
            pose.orientation.z = float(msg_data_fields[6])
            pose.orientation.w = float(msg_data_fields[7])
            self.teammate_poses[msg.sender] = pose


        else:
            print('Unknown message type:' + str(msg_type))

        #print('comm_manager msg received from:' + msg.sender + ' msg:' + msg.msg)
        #print("==================================\n")

    def publish_pose(self):
        msg_string_list = []
        pose = self.uav_controller.get_latest_pose()
        msg_string_list.append(str(POSE_MSG) + ' ')
        msg_string_list.append(str(pose.position.x) + ' ')
        msg_string_list.append(str(pose.position.y) + ' ')
        msg_string_list.append(str(pose.position.z) + ' ')
        msg_string_list.append(str(pose.orientation.x) + ' ')
        msg_string_list.append(str(pose.orientation.y) + ' ')
        msg_string_list.append(str(pose.orientation.z) + ' ')
        msg_string_list.append(str(pose.orientation.w))
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)

    def publish_terrorists(self):
        Terrorists = self.terrorist_detector.terroristArray
        if (Terrorists != []):
            msg_string_list = []
            msg_string_list.append(str(TERRORIST_MSG) + ' ')
            for t in Terrorists:
                if (t != []):
                    msg_string_list.append(str(t[0]) + ' ')  # finder UAV
                    msg_string_list.append(str(t[1]) + ' ')  # terrorist_name
                    msg_string_list.append(str(t[2]) + ' ')  # x
                    msg_string_list.append(str(t[3]) + ' ')  # y
                    msg_string_list.append(str(t[4]) + ' ')  # z
                    msg_string_list.append(str(t[5]) + ' ')  # time
                else:
                    msg_string_list.append('x ')
                    msg_string_list.append('x ')
                    msg_string_list.append('x ')
                    msg_string_list.append('x ')
                    msg_string_list.append('x ')
                    msg_string_list.append('x ')
            uav_msg = UAVMessage()
            uav_msg.sender = self.uav_name
            uav_msg.msg = ''.join(msg_string_list)
            self.msg_publisher.publish(uav_msg)