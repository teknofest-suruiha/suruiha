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

GUIDED = 'GUIDED'
REFUEL = 'REFUEL'
LANDING = 'LANDING'
LANDED = 'LANDED'
TRACK = 'TRACK'

rospy.logdebug('waiting for /air_traffic_control service')
rospy.wait_for_service('/air_traffic_control')
air_traffic_service = rospy.ServiceProxy('/air_traffic_control', AirTraffic)


def traffic_manager(uav, command):
    response = air_traffic_service(uav, command)
    return response.result
