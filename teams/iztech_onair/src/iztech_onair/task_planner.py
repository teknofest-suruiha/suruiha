from iztech_onair.battery_manager import BatteryManager
from geometry_msgs.msg import Pose


import rospy

# task status
TAKEOFF = 'TAKEOFF'
RETAKEOFF = 'RETAKEOFF'
WAYPOINTS = 'WAYPOINTS'
WAYPOINTSLARGER = 'WAYPOINTSLARGER'
LAND = 'LAND'
IDLE = 'IDLE'
SEARCH = 'SEARCH'
SEARCHLARGER = 'SEARCHLARGER'
DETECTOR = 'DETECTOR'
DETECTION = 'DETECTION'


class TaskPlanner:

    def __init__(self, uav_controller, uav_name, uav_index, terrorist_detector):
        global area_size
        if rospy.has_param('scenario'):
            scenario_params = rospy.get_param('scenario')
            area_size = scenario_params['area_size']
            self.num_uavs = scenario_params['num_uavs']
            uavs = scenario_params['uavs']
            areawidth = area_size['width']
            areaheight = area_size['height']
            self.max_height=uavs[uav_index]['sensor']['max_height']
            self.min_height=uavs[uav_index]['sensor']['min_height']
            x = areawidth / 4
            y = areaheight / 4
            z = self.max_height-1
            if self.min_height < 30:
                self.min_height = 30

        else:
            x=300
            y=300

        a = [-x, -y, z]
        b = [x - x, y - y, z]
        c = [x, y, z]
        d = [(2*x)-x/2, y - y, z]
        e = [x, -y, z]
        f = [-x, y, z]
        g = [(-2*x)+x/2, y - y, z]
        h = [x - x, (2*y)-y/2, z]
        j = [x - x, (-2*y)+y/2, z]

        self.last_uav_pose = Pose()
        self.terrorist_detector = terrorist_detector
        self.controller = uav_controller
        self.uav_name = uav_name
        self.uav_index = uav_index
        self.status = TAKEOFF
        self.battery_manager_init = BatteryManager(uav_name)
        self.way_points_zephyr = [[x, y, z], [-x, y, z], [x, -y, z], [-x, -y, z]]
        self.way_points_iris = [[x-x, y-y, (self.max_height-(10*self.max_height/100))], [x-x/2, y-y/2, (self.max_height-(10*self.max_height/100))], [x-3*x/2, y-3*y/2, (self.max_height-(10*self.max_height/100))], [x+x/2, y-y/2, (self.max_height-(10*self.max_height/100))]]
        self.way_points_detector0 = [a, b, c, d, e, b, f, g, a]
        self.way_points_detector1 = [e, a, b, d, c, f, a, j, e]
        self.way_points_detector2 = [c, h, f, b, a, j, e, b, c]
        self.way_points_detector3 = [f, g, a, b, e, c, d, g, f]

        self.track_point_x = None
        self.track_point_y = None

        self.k = 0

        if self.uav_index == 0:
            self.uavindex = 0
        elif self.uav_index == 1:
            self.uavindex = 1
        elif self.uav_index == 2:
            self.uavindex = 0
        elif self.uav_index == 3:
            self.uavindex = 2
        elif self.uav_index == 4:
            self.uavindex = 1
        elif self.uav_index == 5:
            self.uavindex = 3

        if self.num_uavs == 3:
            self.stepsize = 2
        elif self.num_uavs == 4:
            self.stepsize = 3
        elif self.num_uavs == 5:
            self.stepsize = 3
        elif self.num_uavs == 6:
            self.stepsize = 4

        self.detector_index = 0

    def step(self):
        if self.uav_name.find('iris') >= 0:
            self.terrorist_detector.sendDataToIris(self.uav_name)
            temp_x = self.terrorist_detector.terroristTrackingPoint[0]
            temp_y = self.terrorist_detector.terroristTrackingPoint[1]
            if temp_x != 999999 and temp_y != 999999:
                self.track_point_x = temp_x
                self.track_point_y = temp_y
        battery_manager = self.battery_manager_init
        batrem = battery_manager.last_battery.remaining
        batcap = battery_manager.last_battery.capacity
        if batcap == 0 and batrem == 0:
            batcap = 1
            batrem = 1

        if self.status == TAKEOFF:
            print('task planner takeoff')
            if self.uav_name.find('zephyr'):
                completed = self.controller.takeoff(30)
            elif self.uav_name.find('iris'):
                completed = self.controller.takeoff(50)
            if completed:
                self.status = DETECTOR

        elif self.status == RETAKEOFF:
            print('task planner takeoff')
            if self.uav_name.find('zephyr'):
                completed = self.controller.takeoff(30)
            elif self.uav_name.find('iris'):
                completed = self.controller.takeoff(self.min_height + 15)
            if completed:
                self.status = WAYPOINTS

        elif self.status == DETECTOR:
            print('task planner detector')
            if ((batrem / batcap * 100) - (self.stepsize * self.uavindex)) < 25:
                print('battery become low in detection')
                self.status = LAND
            elif self.uav_name.find('zephyr') >= 0:
                if self.uavindex == 0:
                    way_point = self.way_points_detector0[self.detector_index]
                elif self.uavindex == 1:
                    way_point = self.way_points_detector1[self.detector_index]
                elif self.uavindex == 2:
                    way_point = self.way_points_detector2[self.detector_index]
                elif self.uavindex == 3:
                    way_point = self.way_points_detector3[self.detector_index]
                completed2 = self.controller.goto_position(way_point[0], way_point[1], way_point[2]-self.uavindex*2, 600)
                if completed2:
                    self.status = DETECTION
            elif self.uav_name.find('iris') >= 0:
                self.status = WAYPOINTS

        elif self.status == WAYPOINTS:
            print('task planner waypoints')
            if ((batrem / batcap * 100) - (2 * self.stepsize * self.uavindex)) < 25:
                print('battery become low in waypoint')
                self.status = LAND
            elif self.uav_name.find('zephyr') >= 0:
                way_point = self.way_points_zephyr[self.uavindex]
                completed2 = self.controller.goto_position(way_point[0], way_point[1], way_point[2]-self.uavindex*2, 600)
                if completed2:
                    self.status = SEARCH
            elif self.uav_name.find('iris') >= 0:
                if self.track_point_x is None and self.track_point_y is None:
                    way_point = self.way_points_iris[self.uavindex]
                    trackthrottle = 435
                else:
                    print('Now tracking the terrorist at the position : ' + str(self.track_point_x) + ',' + str(self.track_point_y))
                    way_point = [self.track_point_x, self.track_point_y, (self.max_height-(10*self.max_height/100))]
                    trackthrottle = 425
                    if self.last_uav_pose.position.z < 25:
                        trackthrottle = 435
                completed2 = self.controller.goto_position(way_point[0], way_point[1], way_point[2]-self.uavindex*2, trackthrottle, threshold=5)
                if completed2:
                    self.status = SEARCH

        elif self.status == WAYPOINTSLARGER:
            print('task planner waypointslarger')
            if ((batrem / batcap * 100) - (self.stepsize * self.uavindex)) < 25:
                print('battery become low in waypoint')
                self.status = LAND
            elif self.uav_name.find('zephyr') >= 0:
                way_point = self.way_points_zephyr[self.uavindex]
                completed2 = self.controller.goto_position(way_point[0], way_point[1], way_point[2] - self.uavindex * 2, 550)
                if completed2:
                    self.status = SEARCHLARGER

        elif self.status == DETECTION:
            detected = self.controller.detection(self.max_height-5)
            print('task planner detection')
            if ((batrem / batcap * 100) - (2 * self.stepsize * self.uavindex)) < 25:
                print('battery become low in detecting')
                self.status = LAND
            elif detected:
                if self.uavindex == 0:
                    self.detector_index = self.detector_index + 1
                    if self.detector_index <= (len(self.way_points_detector0)-1):
                        self.status = DETECTOR
                    else:
                        self.status = WAYPOINTS
                elif self.uavindex == 1:
                    self.detector_index = self.detector_index + 1
                    if self.detector_index <= (len(self.way_points_detector1)-1):
                        self.status = DETECTOR
                    else:
                        self.status = WAYPOINTS
                elif self.uavindex == 2:
                    self.detector_index = self.detector_index + 1
                    if self.detector_index <= (len(self.way_points_detector2)-1):
                        self.status = DETECTOR
                    else:
                        self.status = WAYPOINTS
                elif self.uavindex == 3:
                    self.detector_index = self.detector_index + 1
                    if self.detector_index <= (len(self.way_points_detector3)-1):
                        self.status = DETECTOR
                    else:
                        self.status = WAYPOINTS

        elif self.status == SEARCH:
            if self.uav_name.find('zephyr') >= 0:
                searched = self.controller.loiter(50, self.max_height-5)
                print('task planner searching')
                if (((batrem / batcap) * 100) - (2 * self.stepsize * self.uavindex)) < 25:
                    print('battery become low in searching')
                    self.status = LAND
                elif searched:
                        self.uavindex = self.uavindex + self.stepsize
                        if self.uavindex <= (len(self.way_points_zephyr) - self.stepsize + 1):
                            self.status = WAYPOINTS
                        else:
                            self.uavindex = 0
                            self.status = SEARCHLARGER

            elif self.uav_name.find('iris') >= 0:
                if self.track_point_x is not None and self.track_point_y is not None:
                    self.status = WAYPOINTS
                else:
                    way_point = self.way_points_iris[self.uavindex]
                    searched = self.controller.loiter(way_point[0], way_point[1], way_point[2])
                    print('task planner searching')
                    if (((batrem / batcap) * 100) - (2 * self.stepsize * self.uavindex)) < 25:
                        print('battery become low in searching')
                        self.status = LAND
                    elif searched:
                        self.uavindex = self.uavindex + self.stepsize
                        if self.uavindex <= (len(self.way_points_iris) - self.stepsize + 1):
                            self.status = WAYPOINTS
                        else:
                            self.status = LAND

        elif self.status == SEARCHLARGER:
            if self.uav_name.find('zephyr') >= 0:
                searched = self.controller.loiter_larger(50, self.max_height-5)
                print('task planner searching larger')
                if (((batrem / batcap) * 100) - (2 * self.stepsize * self.uavindex)) < 25:
                    print('battery become low in searching larger')
                    self.status = LAND
                elif searched:
                        self.uavindex = self.uavindex + 1
                        if self.uavindex <= (len(self.way_points_zephyr) - 1):
                            self.status = WAYPOINTSLARGER
                        else:
                            self.status = LAND

        elif self.status == LAND:
            completed = self.controller.land()
            if completed:
                for x in range(100):
                    print(x)
                    self.controller.stop_motors()
                else:
                    self.status = IDLE

        elif self.status == IDLE:
            print('idle state')
            if self.uav_name.find('zephyr') >= 0:
                if self.uavindex <= (len(self.way_points_zephyr) - self.stepsize + 1):
                    self.status = RETAKEOFF
                else:
                    self.controller.stop_motors()
            elif self.uav_name.find('iris') >= 0:
                if self.uavindex <= (len(self.way_points_iris) - self.stepsize + 1):
                    self.status = RETAKEOFF
                else:
                    self.controller.stop_motors()
        print("\n--------------------------------------\n")


