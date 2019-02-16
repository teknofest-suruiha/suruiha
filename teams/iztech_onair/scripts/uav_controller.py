#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib

roslib.load_manifest('iztech_onair')

import rospy

import sys
from iztech_onair.terrorist_detector import TerroristDetector
from iztech_onair.air_traffic_manager import AirTrafficManager
from iztech_onair.zephyr_controller import ZephyrController
from iztech_onair.iris_controller import IrisController
from iztech_onair.task_planner import TaskPlanner
from iztech_onair.comm_manager import CommManager
from iztech_onair.sensor_manager import SensorManager
from iztech_onair.score_manager import ScoreManager
from iztech_onair.battery_manager import BatteryManager
import iztech_onair.scenario as scenario

if __name__ == "__main__":
    rospy.init_node('uav_controller')
    uav_index = int(sys.argv[1])
    scenario.get_scenario_params()

    if uav_index == 0:
        uav_name = 'zephyr' + str(uav_index)
    elif uav_index == 1:
        uav_name = 'zephyr' + str(uav_index)
    elif uav_index == 2:
        uav_name = 'iris' + str(uav_index)
    elif uav_index == 3:
        uav_name = 'zephyr' + str(uav_index)
    elif uav_index == 4:
        uav_name = 'iris' + str(uav_index)
    elif uav_index == 5:
        uav_name = 'zephyr' + str(uav_index)

    print(uav_name)

    air_manager = AirTrafficManager(uav_name)

    controller = None

    if uav_name.find('zephyr') >= 0:
        controller = ZephyrController(uav_name, air_manager)
    else:
        controller = IrisController(uav_name, air_manager)


    sensor_manager = SensorManager(uav_name)
    terrorist_detector = TerroristDetector(sensor_manager)
    task_planner = TaskPlanner(controller, uav_name, uav_index, terrorist_detector)
    score_manager = ScoreManager()
    battery_manager = BatteryManager(uav_name)
    comm_manager = CommManager(uav_name, controller, terrorist_detector)
    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        score_manager.step()
        battery_manager.step()
        terrorist_detector.step(uav_name)
        task_planner.step()
        # share the latest pose of the uav with other uavs
        #comm_manager.publish_pose()
        comm_manager.publish_terrorists()
        ros_rate.sleep()
