#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib

roslib.load_manifest('vist_team')
import rospy
import sys
from vist_team.terrorist_detector import TerroristDetector
from vist_team.air_traffic_manager import AirTrafficManager
from vist_team.zephyr_controller import ZephyrController
from vist_team.task_planner import TaskPlanner
from vist_team.comm_manager import CommManager
from vist_team.sensor_manager import SensorManager
from vist_team.score_manager import ScoreManager
import vist_team.scenario as scenario

from vist_team.GetPath import GetPath
from vist_team.TerroristPursuit import TerroristPursuit
from vist_team.CaseUpdate import CaseUpdate



if __name__ == "__main__":
    rospy.init_node('uav_controller', anonymous = True)
    uav_index = int(sys.argv[1])
    uav_name = 'zephyr' + str(uav_index)
    mission = "ALAN_TARAMA"
    case = "KALKIS"
    counter = 1

    num_uavs, comm_distance, area_size, runway, uavs, uav_height = scenario.get_scenario_params(uav_index)
    scenario.print_params()

    air_manager = AirTrafficManager(uav_name)
    controller = ZephyrController(uav_index, air_manager,uav_height)

    comm_manager = CommManager(uav_name, uav_index, controller)
    sensor_manager = SensorManager(uav_name)
    path = GetPath(area_size, uav_height, runway, num_uavs,uav_index)[uav_index]
    task_planner = TaskPlanner(controller, uav_name, path, mission, case, area_size["width"])

    terrorist_detector = TerroristDetector(sensor_manager, controller, uav_height)
    #score_manager = ScoreManager()
    case_update = CaseUpdate(uav_name, mission, case, comm_manager, terrorist_detector)
    terrorist_pursuit = TerroristPursuit(mission, case, terrorist_detector, uav_height)


    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        mission, case = case_update.step(mission, case, counter)
        print(mission)
        print(case)
        mission, case = terrorist_detector.step(mission, case, counter)
        mission, case = task_planner.step(mission, case, counter)
        comm_manager.message_transmit(counter)
        #if mission == 'T_TAKIP':
        #path = TerroristPursuit.step(mission, case)
        counter += 1
        ros_rate.sleep()
