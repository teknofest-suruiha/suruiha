# -*- coding: utf-8 -*-
import rospy
from suruiha_gazebo_plugins.msg import UAVBattery

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


class CaseUpdate:
    def __init__(self, uav_name, mission, case, comm_manager, terrorist_detector):

        self.case = case
        self.mission = mission
        self.uav_name = uav_name
        rospy.Subscriber('/' + uav_name + '_battery', UAVBattery, self.message_received)
        self.comm_manager = comm_manager
        self.terrorist_detector = terrorist_detector
        self.battery_ratio = 1
        self.commStatus = True
        self.trDetect = False
        self.battery_limit = 0.2
        self.last_battery = UAVBattery()
        self.counter = 0

    def step(self, gorev, durum, counter):
        self.counter = counter
        self.mission = gorev
        self.case = durum
        if self.last_battery.capacity != 0:
            self.battery_ratio = self.last_battery.remaining / self.last_battery.capacity
        if self.counter %200 == 0:
            self.commStatus = self.comm_manager.check()
        self.trDetect = self.terrorist_detector.check()

        #if not self.commStatus:
            #self.case = H_KOPTU

        if self.trDetect:
            if self.mission != T_TAKIP:
                self.case = T_TESPIT

        #if self.case == T_TESPIT and self.mission != T_TAKIP:
        #   self.mission = T_TAKIP

        elif  self.battery_ratio < self.battery_limit:
            self.case = BATARYA_OLU

        elif self.mission == INIS and self.case == BOS:
            if self.battery_ratio == 1:
                self.case = KALKIS
        return self.mission, self.case

    def message_received(self, battery_msg):
        self.last_battery = battery_msg
