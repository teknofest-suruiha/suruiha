# -*- coding: utf-8 -*-

from suruiha_gazebo_plugins.msg import UAVBattery
import rospy


battery_limit = 0.1

class BatteryManager:
    def __init__(self, drone):
        uav_name = drone.name
        rospy.Subscriber('/'+uav_name+'_battery', UAVBattery, self.message_received)
        self.last_battery = UAVBattery()
        self.battery_ratio = 1

    def message_received(self, battery_msg):
        self.last_battery = battery_msg

    def get_last_battery(self):
        return self.last_battery

    def step(self, drone):
        print('battery_remaining:' + str(self.last_battery.remaining))
        print('battery_capacity:' + str(self.last_battery.capacity))
        
        if self.last_battery.capacity != 0:
            self.battery_ratio = self.last_battery.remaining / self.last_battery.capacity
        


