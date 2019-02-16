import time
import rospy
from suruiha_gazebo_plugins.srv import AirTraffic

# air traffic message constants
STATUS = "STATUS"
TAKEOFF_REQUEST = "TAKEOFF_REQUEST"
LANDING_REQUEST = "LANDING_REQUEST"
CANCEL = "CANCEL"
LANDING_POSE = "LANDING_POSE"

AVAILABLE = "AVAILABLE"
TAKEN = "TAKEN"
ALLOCATED_TO_LAND = "ALLOCATED_TO_LAND"
ALLOCATED_TO_TAKEOFF = "ALLOCATED_TO_TAKEOFF"
READY_TO_TAKEOFF = "READY_TO_TAKEOFF"

# uav status
IDLE = 'IDLE'
WAITING_AIR_TRAFFIC = 'WAITING_AIR_TRAFFIC'
WAITING_TO_TAKEOFF = 'WAITING_TO_TAKEOFF'
TAKEOFF = 'TAKEOFF'
# ONAIR = 'ONAIR'


class AirTrafficManager:
    def __init__(self, uav_name):
        self.uav_name = uav_name

        # connect to air traffic controller
        rospy.logdebug('waiting for /air_traffic_control service')
        rospy.wait_for_service('/air_traffic_control')
        self.air_traffic_service = rospy.ServiceProxy('/air_traffic_control', AirTraffic)

        self.status = IDLE

    def takeoff_request(self):
        if self.status == IDLE:
            response = self.air_traffic_service(self.uav_name, TAKEOFF_REQUEST)
            if response.result == ALLOCATED_TO_TAKEOFF:
                self.status = WAITING_TO_TAKEOFF
            elif response.result == TAKEN:
                self.status = WAITING_AIR_TRAFFIC
        elif self.status == WAITING_AIR_TRAFFIC:
            # print('waiting air traffic')
            response = self.air_traffic_service(self.uav_name, TAKEOFF_REQUEST)
            if response.result == ALLOCATED_TO_TAKEOFF:
                self.status = WAITING_TO_TAKEOFF
        elif self.status == WAITING_TO_TAKEOFF:
            # print('waiting to takeoff')
            response = self.air_traffic_service(self.uav_name, STATUS)
            if response.result == READY_TO_TAKEOFF:
                print('ready to takeoff')
                self.status = TAKEOFF
                return False
        elif self.status == TAKEOFF:
            # print('takeoff state')
            return True

        # waiting air traffic to be able to takeoff
        return False

    def landing_request(self):
        if self.status == TAKEOFF:
            response = self.air_traffic_service(self.uav_name, LANDING_REQUEST)
            print('landing_request result:' + response.result)
            if response.result == ALLOCATED_TO_LAND:
                self.status = ALLOCATED_TO_LAND
        elif self.status == ALLOCATED_TO_LAND:
            return True

        return False

    def get_status(self):
        return self.status

    def set_status(self, new_status):
        self.status = new_status

    def get_landing_pose(self):
        starting_pose = []
        ending_pose = []
        response = self.air_traffic_service(self.uav_name, LANDING_POSE)
        values_str = response.result.strip().split(" ")
        for val_str in values_str:
            if val_str:
                if len(starting_pose) < 6:
                    starting_pose.append(float(val_str))
                else:
                    ending_pose.append(float(val_str))

        if len(starting_pose) != 6 or len(ending_pose) != 6:
            print('landing pose msg is corrupted:' + response.result)

        return starting_pose, ending_pose





