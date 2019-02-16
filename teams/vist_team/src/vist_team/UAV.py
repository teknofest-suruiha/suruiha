# -*- coding: utf-8 -*-

FIELDSEARCH = 'FIELDSEARCH'     # Durumu: ONAIR
T_PURSUIT = 'T_PURSUIT'         # Durumu: ONPURSUIT


ONAIR = 'ONAIR'
ONPURSUIT = 'ONPURSUIT'
T_DETECT = 'T_DETECT'
COMMOFF = 'COMMOFF'
BATTERYDIE = 'BATTERYDIE'



class UAV:
    def __init__(self, index, height):
        self.index = index
        self.name = 'zephyr' + str(self.index)
        self.height = height
        self.path = []
        self.pursuit_point = []        
        self.case = ONAIR
        self.task = FIELDSEARCH
#        self.connection_status = True
        
        self.air_manager        = None
        self.controller         = None
        self.comm_manager       = None
        self.sensor_manager     = None
        self.battery_manager    = None
        self.task_planner       = None
        self.terrorist_detector = None
        self.score_manager      = None
        
        
        
        
        
        