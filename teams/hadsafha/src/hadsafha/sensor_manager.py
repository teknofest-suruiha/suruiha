from suruiha_gazebo_plugins.msg import UAVSensorMessage
import rospy


class SensorManager:
    def __init__(self, uav_name):
        rospy.Subscriber(uav_name + '_sensor', UAVSensorMessage, self.message_received)
        self.last_perception = UAVSensorMessage()

    def message_received(self, sensor_msg):
        self.last_perception = sensor_msg

    def get_last_perception(self):
        return self.last_perception
