import roslib
import rospy

roslib.load_manifest('test_ucusu')


def get_scenario_params():
    if rospy.has_param('scenario'):
        scenario_params = rospy.get_param('scenario')
        num_uavs = scenario_params['num_uavs']
        comm_distance = scenario_params['comm_distance']
        area_size = scenario_params['area_size']
        runway = scenario_params['runway']
        uavs = scenario_params['uavs']
        return num_uavs, comm_distance, area_size, runway, uavs
    else:
        print('Scenario topic can not be found in the server')
        return None