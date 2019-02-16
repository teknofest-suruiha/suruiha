from suruiha_gazebo_plugins.msg import UAVMessage
import rospy

# GOREVLER
ALAN_TARAMA = '0'
TERORIST_TAKIP = '1'
ACIL_INIS = '2'

#DURUMLAR
Alan_Tarama = 'A'
Terorist_Tespit_Edildi = 'B'
Batarya_Dusuk = 'C'
Carpisma_Tehlikesi = 'D'
Kalkis = 'E'
Baglanti_Koptu = 'F'
Bosta = 'G'
Takipte = 'H'
Terorist_Kaybedildi = 'I'
Yeni_Terorist_Tespit_Edildi = 'J'


class Hafiza:
    def __init__(self):
        self.ihanum = 0
        self.indis = 0
        self.gorev = "1"
        self.durum = "A"
        self.konum = [4,5,6]
        self.batarya = 7

class Tlist:
    def __init__(self):
        self.indis = 0
        self.durum = 0
        self.grup = None
        self.grupagort = None
        self.konum = [1,2,3]
        self.yon = [1,2,1,2]

class Blist:
    def __init__(self):
        self.indis = '0'
        self.konum = [1,0,1]


class CommManager:
    def __init__(self, uav_name, uav_index, uav_controller):
        self.uav_name = uav_name
        self.uav_index= uav_index
        self.iha_sayisi = rospy.get_param("/scenario/num_uavs")
        self.msg_publisher = rospy.Publisher('comm_request', UAVMessage, queue_size=10)
        rospy.Subscriber('comm_' + self.uav_name, UAVMessage, self.message_received)
        self.uav_controller = uav_controller
        self.counter = 1
        self.Liste = {}
        self.TeroristListesi = {}
        self.BinaListesi = {}
        self.ListePrev = ""
        self.msgString = ""
        self.BnListesi = {}
        self.TrListesi = {}
        
        for i in range(self.iha_sayisi):
            self.Liste["zephyr" + str(i)] = Hafiza()

    #def GorevDurum(self,):
        #Gorevdagitim ile yazilacak

    def message_transmit(self, counter):
        self.counter = counter
        msg_string_list = []
        pose = self.uav_controller.get_latest_pose()
        #Konumu guncelle
        self.Liste[self.uav_name].konum=[int(pose.position.x),int(pose.position.y),int(pose.position.z)]
        self.Liste[self.uav_name].indis += 1

        #IHA bilgilerini siraya diz
        for n in range(self.iha_sayisi):
            msg_string_list.append("zephyr" + str(n) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].ihanum) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].indis)+' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].gorev) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].durum) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].konum[0]) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].konum[1]) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].konum[2]) + ' ')
            msg_string_list.append(str(self.Liste["zephyr" + str(n)].batarya) + ' ')

        #Terorist verilerini siraya diz
        for p in range(len(self.TeroristListesi)):
            msg_string_list.append("terrorist_")
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].indis) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].durum) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].grup) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].grupagort[0]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].grupagort[1]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].grupagort[2]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].konum[0]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].konum[1]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].konum[2]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].yon[0]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].yon[1]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].yon[2]) + ' ')
            msg_string_list.append(str(self.TeroristListesi["terrorist_" + str(p)].yon[3]) + ' ')

        #Bina verilerini siraya diz
        for k in range(len(self.BinaListesi)):
            msg_string_list.append("bina"+' ')
            msg_string_list.append(str(self.BinaListesi["Bina" + str(k)].indis) + ' ')
            msg_string_list.append(str(self.BinaListesi["Bina" + str(k)].konum[0]) + ' ')
            msg_string_list.append(str(self.BinaListesi["Bina" + str(k)].konum[1]) + ' ')
            msg_string_list.append(str(self.BinaListesi["Bina" + str(k)].konum[2]) + ' ')

        #Olusturulan diziyi gonder
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)
        #return uav_msg.msg

    def message_received(self, msg):
        self.msgString = msg.msg
        self.msg_data_fields = msg.msg.split(' ')
        self.hfzGuncelle(self.msg_data_fields)

    def getList(self):
        return self.Liste


    def check(self):
        if self.counter % 200 == 0:
            if msg.sender:
                return True
            else:
                return True
        #self.counter += 1


    def hfzGuncelle(self, gelenListe):
        self.uListe = gelenListe
        for b in range(self.iha_sayisi):
            if int(self.uListe[9*b+2]) > self.Liste["zephyr" + str(b)].indis:
                self.Liste["zephyr" + str(b)].ihanum = int(self.uListe[9*b+1])
                self.Liste["zephyr" + str(b)].indis = int(self.uListe[9*b+2])
                self.Liste["zephyr" + str(b)].gorev = self.uListe[9*b+3]
                self.Liste["zephyr" + str(b)].durum = self.uListe[9*b+4]
                self.Liste["zephyr" + str(b)].konum[0] = int(self.uListe[9*b+5])
                self.Liste["zephyr" + str(b)].konum[1] = int(self.uListe[9*b+6])
                self.Liste["zephyr" + str(b)].konum[2] = int(self.uListe[9*b+7])
                self.Liste["zephyr" + str(b)].batarya = int(self.uListe[9*b+8])
        if len(self.TeroristListesi) > 0:
            self.TrsGuncelle(self.uListe)
        if len(self.BinaListesi) > 0:
            self.BinaGuncelle(self.uListe)

    #def TrsEkle(self):
        #Terorist tespitle birlikte buralar doldurulacak

    def TrsGuncelle(self, gelenListe):
        self.tind = self.iha_sayisi * 9
        self.tListe = gelenListe[self.tind:]

        self.k = 0
        while self.tListe[14 * self.k]=='terrorist':
            #Listeyi guncelle
            self.TeroristListesi["terrorist_" + str(self.k)].indis = int(self.tListe[14 * self.k+1])
            self.TeroristListesi["terrorist_" + str(self.k)].durum = int(self.tListe[14 * self.k + 2])
            self.TeroristListesi["terrorist_" + str(self.k)].grup = int(self.tListe[14 * self.k + 3])
            self.TeroristListesi["terrorist_" + str(self.k)].grupagort[0] = int(self.tListe[14 * self.k + 4])
            self.TeroristListesi["terrorist_" + str(self.k)].grupagort[1] = int(self.tListe[14 * self.k + 5])
            self.TeroristListesi["terrorist_" + str(self.k)].grupagort[2] = int(self.tListe[14 * self.k + 6])
            self.TeroristListesi["terrorist_" + str(self.k)].konum[0] = int(self.tListe[14 * self.k + 7])
            self.TeroristListesi["terrorist_" + str(self.k)].konum[1] = int(self.tListe[14 * self.k + 8])
            self.TeroristListesi["terrorist_" + str(self.k)].konum[2] = int(self.tListe[14 * self.k + 9])
            self.TeroristListesi["terrorist_" + str(self.k)].yon[0] = int(self.tListe[14 * self.k + 10])
            self.TeroristListesi["terrorist_" + str(self.k)].yon[1] = int(self.tListe[14 * self.k + 11])
            self.TeroristListesi["terrorist_" + str(self.k)].yon[2] = int(self.tListe[14 * self.k + 12])
            self.TeroristListesi["terrorist_" + str(self.k)].yon[3] = int(self.tListe[14 * self.k + 13])

            self.k += 1

    #def BinaEkle(self):
        # Terorist tespitle birlikte buralar doldurulacak

    def BinaGuncelle(self,gelenListe):
        self.bind = self.iha_sayisi * 9 + (len(self.TeroristListesi)) * 14
        self.bListe = gelenListe[self.bind:]
        self.l=0

        while self.bListe[5 * self.l] == 'bina':
            # Listeyi guncelle
            self.BinaListesi["Bina" + str(self.l)].indis = int(self.bListe[5 * self.l]+1)
            self.BinaListesi["Bina" + str(self.l)].konum[0] = int(self.bListe[5 * self.l]+2)
            self.BinaListesi["Bina" + str(self.l)].konum[1] = int(self.bListe[5 * self.l]+3)
            self.BinaListesi["Bina" + str(self.l)].konum[2] = int(self.bListe[5 * self.l]+4)

            self.l += 1
