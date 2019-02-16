from suruiha_gazebo_plugins.msg import UAVSensorMessage
import rospy


class SensorManager:
    def __init__(self, uav_name):
        rospy.Subscriber(uav_name + '_sensor', UAVSensorMessage, self.message_received)
        self.perception = UAVSensorMessage()

    def message_received(self, sensor_msg):
        self.perception = sensor_msg

    def get_perception(self):
        return self.perception
