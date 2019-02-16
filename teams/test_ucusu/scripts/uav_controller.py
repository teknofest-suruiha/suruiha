#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib

roslib.load_manifest('test_ucusu')
import rospy
import sys
from test_ucusu.terrorist_detector import TerroristDetector
from test_ucusu.air_traffic_manager import AirTrafficManager
from test_ucusu.zephyr_controller import ZephyrController
from test_ucusu.task_planner import TaskPlanner
from test_ucusu.comm_manager import CommManager
from test_ucusu.sensor_manager import SensorManager
from test_ucusu.score_manager import ScoreManager
from test_ucusu.battery_manager import BatteryManager
import test_ucusu.scenario as scenario


if __name__ == "__main__":
    rospy.init_node('uav_controller')
    # uav_index = int(sys.argv[1])
    uav_index = 0
    scenario.get_scenario_params()

    uav_name = 'zephyr' + str(uav_index)

    air_manager = AirTrafficManager(uav_name)
    controller = ZephyrController(uav_name, air_manager)
    comm_manager = CommManager(uav_name, controller)
    sensor_manager = SensorManager(uav_name)
    task_planner = TaskPlanner(controller, uav_name)
    terrorist_detector = TerroristDetector(sensor_manager)
    score_manager = ScoreManager()
    battery_manager = BatteryManager(uav_name)

    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        score_manager.step()
        battery_manager.step()
        terrorist_detector.step()
        task_planner.step()
        # share the latest pose of the uav with other uavs
        comm_manager.publish_pose()
        ros_rate.sleep()
