# -*- coding: utf-8 -*-
import rospy

num_uavs = 0
comm_distance = 0
area_size = None
runway = None
uavs = None


def get_scenario_params():
    global num_uavs, comm_distance, area_size, runway, uavs
    if rospy.has_param('scenario'):
        scenario_params = rospy.get_param('scenario')
        num_uavs = scenario_params['num_uavs']
        comm_distance = scenario_params['comm_distance']
        area_size = scenario_params['area_size']
        runway = scenario_params['runway']
        uavs = scenario_params['uavs']
    else:
        print('ERROR: "scenario" parametre sunucusunda bulunamadı')


def print_params():
    print('num_uavs:' + str(num_uavs))
    print('comm_distance:' + str(comm_distance))
    print('area_size.width:' + str(area_size['width']) + ' height:' + str(area_size['height']))
    print('runway.left_bottom.x:' + str(runway['left_bottom']['x']) + ' .y:' + str(runway['left_bottom']['y']))
    print('runway.right_top.x:' + str(runway['right_top']['x']) + ' .y:' + str(runway['right_top']['y']))
    for i in range(len(uavs)):
        print('uav[' + str(i) + ']:')
        print('    index:' + str(uavs[i]['index']))
        print('    type:' + uavs[i]['type'])
        print('    sensor.min_height:' + str(uavs[i]['sensor']['min_height']) +\
              ' max_height:' + str(uavs[i]['sensor']['max_height']))
        print('    battery_capacity:' + str(uavs[i]['battery_capacity']))