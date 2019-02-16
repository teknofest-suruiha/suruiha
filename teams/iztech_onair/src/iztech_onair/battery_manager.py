from suruiha_gazebo_plugins.msg import UAVBattery

import rospy


class BatteryManager:
    def __init__(self, uav_name):
        rospy.Subscriber('/'+uav_name+'_battery', UAVBattery, self.message_received)
        self.last_battery = UAVBattery()

    def message_received(self, battery_msg):
        self.last_battery = battery_msg

    def get_last_battery(self):
        return self.last_battery

    def step(self):
        print('battery_remaining:' + str(self.last_battery.remaining))
        print('battery_capacity:' + str(self.last_battery.capacity))
        

