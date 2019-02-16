EXPLORE = 'kesfet'
FOLLOW = 'takipet'
BATTERY_REFRESH = 'bataryayenile'
LOST = 'kayip'

#import numpy as np
#import cv2
from hadsafha.uav_map import Map
from datetime import datetime, date
from geometry_msgs.msg import Twist, Pose

class AutonomousIntelligence:

    def __init__(self, uav_controller, uav_name, scenario_params, air_manager, comm_manager, sensor_manager, battery_manager, task_planner, terrorist_detector, score_manager):
        self.controller = uav_controller
        self.uav_name = uav_name
        self.comm_manager = comm_manager
        self.sensor_manager = sensor_manager
        self.battery_manager = battery_manager
        self.task_planner = task_planner
        self.terrorist_detector = terrorist_detector
        self.score_manager = score_manager
        self.scenario_params = scenario_params
        self.air_manager = air_manager

        self.status = None



        # Gecici ( IHA'yi monitorize etmek icin )
        area_size = self.scenario_params['area_size']
        self.width = area_size['width']
        self.height = area_size['height']
        #self.monitor_screen = np.zeros((self.width, self.height, 3), np.uint8)

        # Baslangic uav_id'sini sakla
        self.uav_id = uav_controller.get_uav_priority()

        # UAV icin harita olustur
        self.map = Map(self.width, self.height, uav_name)

        # Terorist akip ediliyor mu?
        self.following = False


        # Sensor
        sensor  = self.scenario_params['uavs'][0]['sensor']
        sensor_max = sensor['max_height']
        sensor_min = sensor['min_height']
        sensor_diff = sensor_max - sensor_min
        if sensor_diff < 10:
            self.operating_altitude = sensor_max - sensor_diff / 2
        else:
            self.operating_altitude = sensor_max - 10



    ## Drone icin rota planlamasi yapar
    def plan_route(self):

        self.task_planner.reset_waypoints()

        central_points = self.calculate_uavs_central_points()
        cp_of_uav = central_points[self.controller.uav_priority]

        center_x, center_y = self.convert_coordinate_system_from_width_and_height(cp_of_uav[0], cp_of_uav[1], self.width, self.height)
        start_x, start_y = self.convert_coordinate_system_from_width_and_height(cp_of_uav[2], cp_of_uav[3],self.width,self.height)
        end_x, end_y = self.convert_coordinate_system_from_width_and_height(cp_of_uav[4], cp_of_uav[5], self.width, self.height)
        height = self.operating_altitude
        space = int(self.operating_altitude / 2) #50 onceki degeri

        print("POINTS",central_points)
        print("POINTS[uav]", cp_of_uav)



        route = self.map.slice_area_to_paths(start_x,start_y,end_x,end_y,height,space)

        route = self.map.reverse_map_to_start_from_nearest_point(route, self.controller.get_latest_pose())



        print("ROUTE")
        print(route)
        if route is not None:
            for way_point in route:
                self.task_planner.add_waypoint(way_point)
        '''
        self.task_planner.reset_waypoints()
        self.task_planner.add_waypoint([300, 300, self.operating_altitude])
        self.task_planner.add_waypoint([300, -300, self.operating_altitude])
        '''

    def explore(self):

        # Sense edilen verileri haritaya yerlestir
        self.sense_and_add_map()
        self.task_planner.step()

    def sense_and_add_map(self):
        sensed = self.sensor_manager.get_last_perception()
        # detection yap
        self.terrorist_detector.publish(self.map)
        for index, pose in enumerate(sensed.poses):
            self.map.add_object(sensed.names[index], pose)


    def convert_coordinate_system_from_width_and_height(self, x, y, width, height):

        xp = x - (width / 2)
        yp = (height / 2) - y

        return xp, yp

    def convert_coordinate_system(self, pose, width, height):

        '''
        Aldigi ros koordinatini, opencv coordinatina cevirir
        :param x:
        :param y:
        :return:
        '''
        x = width / 2 + int(pose.position.x)
        y = height / 2 - int(pose.position.y)
        return x, y

    def visual_monitor(self):

        '''
        Hava araclaini daha kolay takip edebilmek icin olusturulmus bir monitor moduludur.
        :return:
        '''
        teammate_poses = self.comm_manager.get_team_mate_poses()

        pose = self.controller.get_latest_pose()

        x, y = self.convert_coordinate_system(pose, self.width, self.height)

        self.monitor_screen = cv2.circle(self.monitor_screen, (x, y), 3, (0, 255, 0), -1)

        tmp_img = self.monitor_screen
        # Takim Arkadaslarini goster
        for i in teammate_poses:
            pose = teammate_poses[i]
            xt, yt = self.convert_coordinate_system(pose, self.width, self.height)
            tmp_img = cv2.circle(tmp_img, (xt, yt), 3, (0, 0, 255), -1)


        # Map uzerindeki verileri bas
        detected_objects = self.map.get_all_detected_objects()
        for obj in detected_objects:

            color = (255, 255, 255)

            if "terrorist" in obj:
                color = (0, 50, 100)
            elif "building" in obj:
                color = (255, 0, 0)

            pose = detected_objects[obj][-1].get('pose')

            xt, yt = self.convert_coordinate_system(pose, self.width, self.height)
            tmp_img = cv2.circle(tmp_img, (xt, yt), 3, color, 2)

        cv2.imshow('img', tmp_img)
        cv2.waitKey(1);

    def step(self):

        #self.visual_monitor()

        battery = self.battery_manager.get_battery_percentage()
        print("[AI]["+str(self.status)+"]............................................|")

        # Kim oldugunu paylas
        self.comm_manager.publish_handshake()
        # Haritayi paylas
        self.comm_manager.publish_map(self.map)
        # Haritayi senkronize et
        self.map.sync_map(self.comm_manager.teammate_detected_objects)

        # Id degisti ise gorevi guncelle
        tmp_id = self.controller.uav_priority
        if self.uav_id != tmp_id:
            self.uav_id = tmp_id
            self.plan_route()


        if battery < 0.3:
            # Gorev ihtiyaci ile iliskilendirilmeli
            recent_status = self.status
            self.status = BATTERY_REFRESH

        if self.status is None:
            self.status = EXPLORE
        elif self.status == EXPLORE:
            if self.map.terrorist_detected:
                self.status = FOLLOW

            self.explore()

        elif self.status == FOLLOW:
            self.sense_and_add_map()
            following_obj_name = self.map.terrorist_detected
            self.follow_terrorist(following_obj_name)


        elif self.status == BATTERY_REFRESH:
            '''
            completed = self.controller.goto_position()
            if res:
                status == recentStatus
            '''
            print("Battery required...")

    def follow_terrorist(self, following_obj_name):

        stop = self.map.get_newest_detected_object(following_obj_name, -1)
        start = self.map.get_newest_detected_object(following_obj_name, 0)

        if self.following is False:
            if start is None:
                print("Cannot follow object because of less viewing\nReverting to Explore")
                self.status = EXPLORE
                return

            self.following = True

        else:
            if start is not None:
                xp, yp = self.predict_by_coordinates_and_time(start, stop)
                print("PREDICTION:", xp, yp)
                print("Current:", self.controller.get_latest_pose())

                self.controller.goto_position(xp, yp, self.operating_altitude, 350)


    def predict_by_coordinates_and_time(self, start, stop):
        now = datetime.now().time()
        diff_btwn_now_and_stop = self.time_difference_seconds(stop["time"], now)
        diff_x = stop["pose"].position.x - start["pose"].position.x
        diff_y = stop["pose"].position.y - start["pose"].position.y
        diff_time = self.time_difference_seconds(start["time"], stop["time"])

        if diff_time is 0:
            print("diff-time is zero. Changing to default value 1")
            diff_time = 1

        pred_x = stop["pose"].position.x + diff_btwn_now_and_stop*2 * (diff_x / diff_time)
        pred_y = stop["pose"].position.y + diff_btwn_now_and_stop*2 * (diff_y / diff_time)

        return pred_x, pred_y

    def time_difference_seconds(self,start,stop):
        return (datetime.combine(date.today(), stop) - datetime.combine(date.today(), start)).seconds

    def is_remaining_enough_battery(self,current_pose, target_pose, landing_pose):
        '''
        Simdi bulunan konumdan targete gidip merkeze donecek kadar batarya varsa true doner
        :param target:
        :return:
        '''
        c_btwn_t = (target_pose[0] - current_pose.position.x)**2 + (target_pose[1] - current_pose.position.y)**2
        t_btwn_l = (landing_pose.position.x - target_pose[0])**2 + (landing_pose.position.x - target_pose[1])**2
        total_length = c_btwn_t + t_btwn_l
        # TODO: Bu kisimlar henuz gelistirilmedi
        #rate = (battery_last - battery_now) / ((current_pose.position.x - past_pose.poisiton.x)**2 + (current_pose.position.y - past_pose.position.y)**2)

        #if battery_now - total_length * rate < 0:
        #    self.status = BATTERY_REFRESH


    def calculate_uavs_central_points(self):
        '''
        Hava aracinin gorevlendirilecegi alani ve alanin merkezini belirler
        :return:
        '''
        coordinate_list = []
        constant = self.scenario_params['num_uavs'] / 2
        divide_w = self.width / (constant * 2)  # i
        divide_h = self.height / 4  # j

        print("d_w=", divide_w)
        print("d_h=", divide_h)
        for i in range(0, constant * 2):
            if i % 2 == 1:
                for j in range(0, 4):
                    if j % 2 == 1:
                        start_x = (i - 1) * divide_w
                        start_y = (j - 1) * divide_h
                        end_x = (i + 1) * divide_w
                        end_y = (j + 1) * divide_h
                        center_x = (i) * divide_w
                        center_y = (j) * divide_h

                        struct = [center_x,
                                  center_y,
                                  start_x,
                                  start_y,
                                  end_x,
                                  end_y
                                  ]

                        coordinate_list.append(struct)

        return coordinate_list
