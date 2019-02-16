import rospy
from suruiha_gazebo_plugins.msg import UAVBattery


class BatteryManager:
    def __init__(self, uav_name):
        rospy.Subscriber('/'+uav_name+'_battery', UAVBattery, self.message_received)
        self.last_battery = UAVBattery()

    def message_received(self, battery_msg):
        self.last_battery = battery_msg

    def get_remaining_battery(self):
        return self.last_battery.remaining

    def get_capacity_battery(self):
        return self.last_battery.capacity
