# -*- coding: utf-8 -*-


ALAN_TARAMA = 'ALAN_TARAMA'
T_TAKIP = 'T_TAKIP'

KALKIS = 'KALKIS'
ALAN_TARAMA = "ALAN_TARAMA"
T_TESPIT = "T_TESPIT"
BATARYA_OLU = "BATARYA_OLU"
TAKIPTE = "TAKIPTE"
H_KOPTU = "H_KOPTU"
T_KAYIP = "T_KAYIP"
T_YENI_TESPIT = "T_YENI_TESPIT"
CARP_TEHLIKE = "CARP_TEHLIKE"

INIS = 'INIS'
BOS = 'BOS'



class TerroristPursuit:
    def __init__(self, mission, case, terrorist_detector, height):
        self.mission = mission
        self.case = case
        self.terrorist_detector = terrorist_detector
        self.height = height
        self.konum = []
        self.path = []


    def step(self):
        self.konum = self.terrorist_detector.agort_al()
        self.path = [self.konum[0], self.konum[1], self.height]
        
        # Aslında BURAYA Bir Loiter hareketi gelmesi gerekir....
