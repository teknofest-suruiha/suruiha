#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib

roslib.load_manifest('hadsafha')
import rospy
import sys
from hadsafha.terrorist_detector import TerroristDetector
from hadsafha.air_traffic_manager import AirTrafficManager
from hadsafha.zephyr_controller import ZephyrController
from hadsafha.iris_controller import IrisController
from hadsafha.task_planner import TaskPlanner
from hadsafha.comm_manager import CommManager
from hadsafha.sensor_manager import SensorManager
from hadsafha.score_manager import ScoreManager
from hadsafha.battery_manager import BatteryManager
from hadsafha.autonomous_intelligence import AutonomousIntelligence
import hadsafha.scenario as scenario

from datetime import datetime


if __name__ == "__main__":
    rospy.init_node('uav_controller')


    # Alınan arguman "_" karakteri ile parçalanır ve bu sayede uav tipi ve index'i seçilmiş olur
    #args= sys.argv[1].split("_")

    #uav_index = int(args[0])
    #uav_type = args[1]
    uav_index = int(sys.argv[1])
    uav_type = 'zephyr'

    scenario_params = scenario.get_scenario_params()

    uav_name = 'iris' + str(uav_index)
    if uav_type == "zephyr":
        uav_name = 'zephyr' + str(uav_index)

    air_manager = AirTrafficManager(uav_name)

    controller = None
    uav_wakeup_time = datetime.now().time()

    if uav_name.find('zephyr') >= 0:
        controller = ZephyrController(uav_name, air_manager, uav_wakeup_time)
    elif uav_name.find('iris') >= 0:
        # Kullanmak için , uav_wakeup_time parametresini eklemeyi unutma
        controller = IrisController(uav_name, air_manager)

    comm_manager = CommManager(uav_name, controller)
    sensor_manager = SensorManager(uav_name)

    battery_manager = BatteryManager(uav_name)
    task_planner = TaskPlanner(controller, uav_name)
    terrorist_detector = TerroristDetector(sensor_manager)
    score_manager = ScoreManager()

    # Autonomous Intelligence
    auto_intelligence = AutonomousIntelligence(controller, uav_name, scenario_params, air_manager, comm_manager, sensor_manager, battery_manager, task_planner, terrorist_detector, score_manager)
    auto_intelligence.plan_route()

    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        auto_intelligence.step()



        # share the latest pose of the uav with other uavs
        comm_manager.publish_pose()
        score_manager.step()
        #print(scenario_params)
        '''
        score_manager.step()
        battery_manager.step()
        terrorist_detector.step()
        task_planner.step()
        
        
        '''
        ros_rate.sleep()

