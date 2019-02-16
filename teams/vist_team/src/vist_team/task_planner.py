# -*- coding: utf-8 -*-

# task status
ALAN_TARAMA = 'ALAN_TARAMA'
T_TAKIP = 'T_TAKIP'

KALKIS = 'KALKIS'
ALAN_TARAMA = "ALAN_TARAMA"
T_TESPIT = "T_TESPIT"
BATARYA_OLU = "BATARYA_OLU"
TAKIPTE = "TAKIPTE"
H_KOPTU = "H_KOPTU"
T_KAYIP = "T_KAYIP"
CARP_TEHLIKE = "CARP_TEHLIKE"
INIS = 'INIS'
BOS = 'BOS'

battery_limit = 0.1


class TaskPlanner:

    def __init__(self, uav_controller, uav_name, way_points, mission, case, kenar):
        self.controller = uav_controller
        self.uav_name = uav_name
        self.mission = mission
        self.case = case
        self.kenar = kenar
        self.waypoint_counter = 0
        #self.way_points = [[300, -300, 95], [300, 300, 95], [-300, 300, 95]]
        self.way_points = way_points
        self.storeWaypoint = 0
        self.counter = 0

    def step(self, mission, case, counter):
        self.counter = counter
        self.mission = mission
        self.case = case
        self.counter = 0
        if self.case == "KALKIS":
            # print('task planner takeoff')
            completed = self.controller.takeoff(30)
            if completed:
                self.case = ALAN_TARAMA
                self.waypoint_counter = 0
                
        if self.mission == ALAN_TARAMA and (self.case == ALAN_TARAMA or self.case == H_KOPTU):
            print('task planner waypoints')
            if self.storeWaypoint > 0:
                self.waypoint_counter = self.storeWaypoint
            way_point = self.way_points[self.waypoint_counter]
            completed = False
            if self.kenar > 750:
                if self.uav_name.find('zephyr') >= 0:
                    completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 900, 90, 0.6)
            if self.kenar <=750:
                if self.uav_name.find('zephyr') >= 0:
                    completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 700, 54, 0.5)
                
            if completed:
                self.waypoint_counter += 1
                if self.waypoint_counter == len(self.way_points):
                    self.waypoint_counter = 0
            
        if self.mission == ALAN_TARAMA and self.case == BATARYA_OLU:
            self.storeWaypoint = self.waypoint_counter
            self.mission = INIS

        if self.mission == T_TAKIP and self.case == T_KAYIP:
            self.mission = ALAN_TARAMA
            self.case = ALAN_TARAMA

        if self.mission == T_TAKIP and self.case == BATARYA_OLU:
            self.mission = INIS

        if self.mission == INIS and self.case == BATARYA_OLU:
            completed = self.controller.land()
            if completed:
                self.case = BOS


        if self.case == BOS:
            print('idle state')
            self.controller.stop_motors()
            # pass
            # do nothing
        #self.counter += 1
        return self.mission, self.case







