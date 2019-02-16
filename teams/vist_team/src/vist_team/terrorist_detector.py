import rospy
from suruiha_gazebo_plugins.msg import UAVTracking
from vist_team.util import distance
from std_msgs.msg import String
SINIR_DEGER = 40
AYRILMA_SINIRI = 40
DIF = 20

# task status
ALAN_TARAMA = 'ALAN_TARAMA'
T_TAKIP = 'T_TAKIP'

KALKIS = 'KALKIS'
T_TESPIT = "T_TESPIT"
BATARYA_OLU = "BATARYA_OLU"
TAKIPTE = "TAKIPTE"
H_KOPTU = "H_KOPTU"
T_KAYIP = "T_KAYIP"
CARP_TEHLIKE = "CARP_TEHLIKE"
INIS = 'INIS'
BOS = 'BOS'

class Teroristlistesi:
    def __init__(self):
        self.indis = 0
        self.durum = 0
        self.grup = None
        self.grupagort = [0,0,0]
        self.konum = [1, 2, 3]
        self.yon = [1, 2, 1, 2]

class Binalistesi:
    def __init__(self):
        self.indis = ""
        self.konum = []

class TerroristDetector:
    def __init__(self, sensor_manager,controller, height):
        self.terrorist_tracking_publisher = rospy.Publisher('/terrorist_tracking', UAVTracking, queue_size=1)
        self.terrorist_detection_publisher = rospy.Publisher('/terrorist_detection', String, queue_size=1)
        self.controller = controller
        self.sensor_manager = sensor_manager
        self.counter = 1
        self.count = 1
        self.height = height
        self.TrList = {}
        self.TrListPrev = {}
        self.BnList = {}
        self.katalog = []
        self.katalogPrev =[]
        # Gecici olanlar
        self.mesafeler = []
        self.secicikoor = []
        self.grupUyeleri = []
        self.grupList = []
        self.grupindisi = 0
        self.agort = []
        self.GrupAgOrt = []
        self.ksum = [0,0,0]
        self.test = []
        self.konumT = []
        self.hareket = 1
        self.tmp = None
        self.tmp2 = None
        self.cek = False
        self.neyse = False
        self.a = []
        self.b = []
        self.birakma = 0
        self.lon = 0
        self.agortPrev = []
        self.differencex = 0
        self.differencey = 0
        self.karargahMsg = []
        self.terrMsg = []
        self.supBina = []
        self.supBinaPrev = []
        self.supTerr = []
        self.supTerrPrev = []
        self.mesafe = []
        self.building = 0
        self.terrorist = 0


    def step(self, mission, case, counter):
        self.counter = counter
        self.mission = mission
        self.case = case
        perception = self.sensor_manager.get_last_perception()

        detectionMsg = UAVTracking()
        trackingMsg = UAVTracking()
        BuildingMsg = UAVTracking()
        print(perception.names)
        # Teroristleri ekle
        for i in range(len(perception.types)):
            if perception.names[i].find('terrorist') >= 0:
                trackingMsg.names.append(perception.names[i])
                trackingMsg.poses.append(perception.poses[i])

            if perception.names[i].find('building') >= 0:
                BuildingMsg.names.append(perception.names[i])
                BuildingMsg.poses.append(perception.poses[i])

        if len(trackingMsg.names) > 0:
            self.terrorist_tracking_publisher.publish(trackingMsg)
        if len(trackingMsg.names) > 0:
            self.cek =True
        else:
            self.cek = False
        for i in range(len(trackingMsg.names)):
            self.tmp = trackingMsg.names[i]
            self.tmp2 = self.tmp.split('_')

            if trackingMsg.names[i] not in self.TrList:
                self.TrList[trackingMsg.names[i]] = Teroristlistesi()
                self.katalog.append(self.tmp2[1])

            self.TrList[trackingMsg.names[i]].indis = int(self.tmp2[1])
            self.TrList[trackingMsg.names[i]].konum[0] = int(trackingMsg.poses[i].position.x)
            self.TrList[trackingMsg.names[i]].konum[1] = int(trackingMsg.poses[i].position.y)
            self.TrList[trackingMsg.names[i]].konum[2] = int(trackingMsg.poses[i].position.z)
            self.TrList[trackingMsg.names[i]].yon[0] = trackingMsg.poses[i].orientation.x
            self.TrList[trackingMsg.names[i]].yon[1] = trackingMsg.poses[i].orientation.y
            self.TrList[trackingMsg.names[i]].yon[2] = trackingMsg.poses[i].orientation.z
            self.TrList[trackingMsg.names[i]].yon[3] = trackingMsg.poses[i].orientation.w
        print(self.katalog)

            # Binalari dict e ekle
        #for i in range(len(BuildingMsg.names)):
        #   tmp3 = BuildingMsg.names[i]
        #    tmp4 = tmp3.split('_')
        #    if BuildingMsg.names[i] not in self.BnList:
        #        self.BnList[BuildingMsg.names[i]] = Binalistesi()
        #    self.BnList[BuildingMsg.names[i]].indis = tmp4[1]
        #    self.BnList[BuildingMsg.names[i]].konum[0] = int(BuildingMsg.poses[i].position.x)
        #    self.BnList[BuildingMsg.names[i]].konum[1] = int(BuildingMsg.poses[i].position.y)
        #    self.BnList[BuildingMsg.names[i]].konum[2] = int(BuildingMsg.poses[i].position.z)

        # Teroristleri grupla
        ######################################################
        if self.counter % 10 == 0 and bool(self.TrList):
            for z in self.katalog:
                if self.TrList["terrorist_" + str(z)].grup is None:
                    self.TrList["terrorist_" + str(z)].grup = self.grupindisi
                    self.birakma += 1
                    self.agort.append(z)
            if self.birakma > 0:
                self.grupindisi += 1
                self.birakma = 0
            print(self.agort)

        # Gruplari takip et
        ######################################################
        # Agirlikli ortalamalari hesaplama
        # Saniyede 4 defa donecek
        if self.counter % 2 == 0 and len(self.agort)>0:
            # Gruplari ayni olanlarin agirlikli ortalamasini al
            self.lon = len(self.agort)
            self.ksum = [0,0,0]
            for t in self.agort:
                self.ksum[0] += self.TrList["terrorist_" + str(t)].konum[0]
                self.ksum[1] += self.TrList["terrorist_" + str(t)].konum[1]
                self.ksum[2] += self.TrList["terrorist_" + str(t)].konum[2]
            print(self.ksum)
            print(self.lon)
            print(self.ksum[0]/self.lon)

            self.konumT= [self.ksum[0] / self.lon, self.ksum[1] / self.lon, self.height]
            self.GrupAgOrt = self.konumT
            del self.ksum[:]
            print(self.konumT)
            for j in self.katalog:
                if self.TrList["terrorist_" + str(j)].indis in self.agort:
                    self.TrList["terrorist_" + str(j)].grupagort[0] = self.GrupAgOrt[0]
                    self.TrList["terrorist_" + str(j)].grupagort[1] = self.GrupAgOrt[1]
                    self.TrList["terrorist_" + str(j)].grupagort[2] = self.GrupAgOrt[2]
            print("GrupAgort: "+str(self.GrupAgOrt))
            self.test = self.GrupAgOrt
            print("test: "+str(self.test))

        if self.mission == ALAN_TARAMA and self.case == T_TESPIT:
            self.case = TAKIPTE
            print("case test1:"+self.case)
        if self.mission == ALAN_TARAMA and self.case == TAKIPTE:
            self.mission = T_TAKIP

            print("case test2:"+self.case)
        elif self.mission == T_TAKIP and self.case == T_TESPIT:
            print("test3: "+str(self.test))
            self.case = TAKIPTE

        elif self.mission == T_TAKIP and self.case == TAKIPTE:
            print("test icerde:")
            print(self.test)
            print(self.test)
            if self.counter % 21 == 0:
                self.agortPrev = self.test
            if self.counter % 39 == 0:
                if self.agortPrev:
                    self.differencex =  self.test[0] - self.agortPrev[0]
                    self.differencey =  self.test[1] - self.agortPrev[1]
            print("difference: "+str(self.differencex)+" "+str(self.differencey))
            if self.counter % 400 == 0:
                if self.differencey == 0 and self.differencex == 0:
                    self.case = T_KAYIP
            if self.counter % 40 == 0:
                print("DESTINATION: "+str(self.test[0]+DIF*self.differencex)+" "+str(self.test[1]+DIF*self.differencey))
                self.neyse = self.controller.goto_position(self.test[0]+DIF*self.differencex, self.test[1]+DIF*self.differencey, self.test[2], 450, 5, 0.8)
        if self.mission == T_TAKIP and self.case == T_KAYIP:
            del self.agort[:]
      




        return self.mission, self.case
        ######################################################


        # Karargah tespiti kisminda yapilacak
        # if len(trackingMsg.names) > 0:
        #   detectionMsg = String()
        #   detectionMsg.data = 'building_1'
        #   self.terrorist_detection_publisher.publish(detectionMsg)

    # def ListeyiGuncelle(self):
    def TeroristListesiniDisariAktar(self):
        return self.TrList

    def BinaListesiniDisariAktar(self):
        return self.BnList

    def check(self):
        return self.cek

    def agort_al(self):
        return self.konumT


