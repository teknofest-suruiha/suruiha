import rospy, math
from suruiha_gazebo_plugins.msg import UAVTracking
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from iztech_onair.util import distance


class TerroristDetector:

    def __init__(self, sensor_manager):
        self.terrorist_tracking_publisher = rospy.Publisher('/terrorist_tracking', UAVTracking, queue_size=1)
        self.terrorist_detection_publisher = rospy.Publisher('/terrorist_detection', String, queue_size=1)
        self.sensor_manager = sensor_manager
        self.detectedTerrorists = []  # tespit edilen teroristlerin numaralari
        self.terroristArray = []  # teroristlerin bilgileri
        self.detectedBuildings = []  # tespit edilen tum binalarinin numaralari
        self.buildingArray = []  # tespit edilen tum binalarin bilgileri
        self.terroristBuildings = []  # tespit edilen terorist binalarinin numaralari
        self.last_uav_pose = Pose()
        self.terroristTrackingPoint = [999999, 999999]
        self.distanceToBuilding=[]
        self.t_b_Names=[]
        self.t_Names=[]
        self.numberOfTB=0
        self.flag = False

    def step(self, uavName):
        perception = self.sensor_manager.get_last_perception()
        trackingMsg = UAVTracking()
        detectionMsg = String()
        for i in range(len(perception.names)):
            if perception.names[i].find('terrorist') >= 0:
                trackingMsg.names.append(perception.names[i])
                trackingMsg.poses.append(perception.poses[i])
            elif perception.names[i].find('building') >= 0:
                for j in range(len(perception.names)):
                    if perception.names[j].find('terrorist') >= 0:
                        tDistance=distance(float(perception.poses[j].position.x),float(perception.poses[j].position.y),0.0,float(perception.poses[i].position.x),float(perception.poses[i].position.y),0.0)
                        if tDistance <= 3: #####################################################################
                            print("5 metre icinde")
                        
                            if [perception.names[j],perception.names[i]] not in self.t_b_Names :
                                self.t_b_Names.append([perception.names[j],perception.names[i]]) 
                                self.distanceToBuilding.append(tDistance) 
                                
                            else:
                                
                                if tDistance > self.distanceToBuilding[self.t_b_Names.index([perception.names[j],perception.names[i]])]:
                                    detectionMsg.data = perception.names[i]
                                    if len(self.t_Names) == 0:
                                        self.terrorist_detection_publisher.publish(detectionMsg)
                                        self.t_Names.append(detectionMsg.data)   
                                    else:
                                        for s in range(len(self.t_Names)):
                                            if len(detectionMsg.data) and detectionMsg.data not in self.t_Names:
                                                self.terrorist_detection_publisher.publish(detectionMsg)
                                                self.t_Names.append(detectionMsg.data)                                 
                                                print('bina: ' + str(detectionMsg))
                                                break

        if len(trackingMsg.names):
            self.terrorist_tracking_publisher.publish(trackingMsg)
            self.savingTerrorist(trackingMsg, self.detectedTerrorists, self.terroristArray, uavName)
        print("numberOfTB: "+str(self.numberOfTB))


    def savingTerrorist(self, message, detected, array, uavName):
        no = -1
        for i in range(len(message.names)):
            no = int(message.names[i].split("_")[1])
            if no not in detected:
                if no >= 0:
                    detected.append(no)
                    if no < len(array):
                        array[no] = [uavName, message.names[i], float(message.poses[i].position.x),
                                     float(message.poses[i].position.y), float(message.poses[i].position.z),
                                     rospy.get_time()]

                    else:
                        for indis in range(len(array), no):
                            array.insert(indis, [])
                        array.insert(no, [uavName, message.names[i], float(message.poses[i].position.x), float(message.poses[i].position.y), float(message.poses[i].position.z), rospy.get_time()])

            else:
                 if no >= 0:  ###kaydedilmis teroristin guncel konumu gelirse
                      array[no] = [uavName, message.names[i], float(message.poses[i].position.x),
                                     float(message.poses[i].position.y), float(message.poses[i].position.z),
                                     rospy.get_time()]

    """def savingBuilding(self, message, detected, array, uavName):
        no = -1
        for i in range(len(message.names)):
            no = int(message.names[i].split("_")[1])
            if no not in detected and no >= 0:
                detected.append(no)
                if no < len(array):
                    array[no] = [uavName, message.names[i], float(message.poses[i].position.x),
                                     float(message.poses[i].position.y), float(message.poses[i].position.z),
                                     rospy.get_time()]
                else:
                    for indis in range(len(array), no):
                        array.insert(indis, [])
                    array.insert(no, [uavName, message.names[i], float(message.poses[i].position.x),
                                      float(message.poses[i].position.y), float(message.poses[i].position.z),
                                      rospy.get_time()])"""

    """def is_terrorist_building(self, perception, t, detectionMsg): 
        flag = False
        if len(tArray):
            for i in range(len(tArray)):
                if tArray[i] != [] and flag == False and math.sqrt(
                        (float(tArray[i][2]) - float(perception.poses[t].position.x)) ** 2.0 + (
                                float(tArray[i][3]) - float(perception.poses[t].position.y)) ** 2.0 + (
                                float(tArray[i][4]) - float(perception.poses[t].position.z)) ** 2.0) < 2:
                    self.terroristBuildings.append(int(perception.names[t].split("_")[1]))
                    detectionMsg.data = perception.names[t]
                    self.terrorist_detection_publisher.publish(detectionMsg)
                    print('published message' + str(detectionMsg))
                    flag = True
                    break
        if flag == False:
        for i in range(len(perception.types)):
            print("perceptiona girdim:\n"+str(perception.names)) # son perception'da hem bina hem terorist konumu varsa bu bina terorist binasidir
            if perception.names[i].find('terrorist') >= 0:
                print("perception icinde terrorist var")
                tDistance=math.sqrt((float(perception.poses[i].position.x) - float(perception.poses[t].position.x)) ** 2.0 + (
                            float(perception.poses[i].position.y) - float(perception.poses[t].position.y)) ** 2.0 )
                print("distance"+str(tDistance))
                if 0.3<tDistance <= 5: 
                    print("5 metre icinde")
                    if perception.names[i] not in self.lastPosition.keys():
                        self.lastPosition[perception.names[i]]=tDistance
                        print("112.satir")
                    else:
                        print("116.satir")
                        if tDistance > self.lastPosition[perception.names[i]]:
                            print("118.satir")
                            self.numberOfTB=self.numberOfTB+1
                            print("tDistance: "+str(tDistance))
                            
                            detectionMsg.data = perception.names[t]
                            self.terrorist_detection_publisher.publish(detectionMsg)
                            #self.terroristBuildings.append(int(perception.names[t].split("_")[1]))
                            print('bina: ' + str(detectionMsg))"""
                    
    """def is_terrorist_building2(self, bArray, perception, t,
                               detectionMsg):  # yeni bir terorist geldiginde onceden bulunan butun binalarin konumlari ile o teroristin konumlarini karsilastirir
        if len(bArray):
            for i in range(len(bArray)):
                if bArray[i] != [] and int(bArray[i][1].split("_")[1]) not in self.terroristBuildings and math.sqrt(
                        (float(bArray[i][2]) - float(perception.poses[t].position.x)) ** 2.0 + (
                                float(bArray[i][3]) - float(perception.poses[t].position.y)) ** 2.0 + (
                                float(bArray[i][4]) - float(perception.poses[t].position.z)) ** 2.0) < 2:
                    self.terroristBuildings.append(int(bArray[i][1].split("_")[1]))
                    detectionMsg.data = bArray[i][1]
                    self.terrorist_detection_publisher.publish(detectionMsg)
                    #print("\ndetectionMsg: " + str(detectionMsg))  ####################################
"""
    def combineLists(self, myList, publicList):  # iki liste alip myList'i zamana gore guncelliyor
         smallList = []
         bigList = []
         if len(myList) == 0:  # eger myList bos ise publicList direk kopyalanir
             myList = publicList
             return myList
         else:
             if len(myList) <= len(publicList):
                 smallList = myList
                 bigList = publicList
             else:
                 smallList = publicList
                 bigList = myList

             for s in range(len(smallList)):
                 if smallList[s] != [] and bigList[s] != []:  # eger iki listede ayni indisli elemanin bilgisi varsa
                     if float(smallList[s][5]) < float(bigList[s][5]):  # eger smallList'teki zaman daha eskiyse bilgileri guncelle
                         smallList[s] = bigList[s]
                 elif smallList[s] == []:  # eger smallList'te o teroristin bilgisi yok ve public'te varsa (tam tersi durumda race condition yaratmamak icin bu asamada bir sey yapilmaz)
                      smallList[s] = bigList[s]

             for b in range(len(smallList), len(bigList)):  # bigList'te olan ama smallList'te olmayan terorist bilgilerini smallList'e kopyalar
                 smallList.append(bigList[b])

             myList = smallList  # smallList'teki bilgiler ile myList'i gunceller
             return myList

    def sendDataToIris(self, uavName):
        if uavName.find('iris') >= 0 and len( self.terroristArray):
            x = y = latest_time = counter = 0.0
            x2 = y2 = latest_time2 = counter2 = 0.0
            x0 = y0 = latest_time0 = counter0 = 0.0
            x4 = y4 = latest_time4 = counter4 = 0.0
            x5 = y5 = latest_time5 = counter5 = 0.0
            dist1 = dist2 = dist3 = dist4 = dist5 = 99999999
            flag = False
            if not flag and uavName.find('iris') >= 0 and len(self.terroristArray):
                for t in range(len(self.terroristArray)):  # terorist konumlarindan en guncel olanini bul
                    if self.terroristArray[t] != [] and uavName == self.terroristArray[t][0] and latest_time < self.terroristArray[t][5]:
                        latest_time = self.terroristArray[t][5]
                for i in range(len(self.terroristArray)):
                    if self.terroristArray[i] != [] and uavName == self.terroristArray[i][0] and (latest_time - self.terroristArray[i][5]) <= 5:
                        counter += 1
                        x += self.terroristArray[i][2]
                        y += self.terroristArray[i][3]
                        flag = True
                if not counter == 0:
                    x /= counter
                    y /= counter
                    dist1 = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, 0, x, y, 0)
            if not flag:
                for r in range(len(self.terroristArray)):
                    if self.terroristArray[r] != [] and self.terroristArray[r][0] == 'zephyr0' and latest_time0 < self.terroristArray[r][5] :
                        latest_time0 = self.terroristArray[r][5]
                for s in range(len(self.terroristArray)):
                    if self.terroristArray[s] != [] and self.terroristArray[s][0] == 'zephyr0' and (latest_time0 - self.terroristArray[s][5]) <= 5 :
                        print('zephyr0''dan konumu aldim.')
                        counter0 += 1
                        x0 += self.terroristArray[s][2]
                        y0 += self.terroristArray[s][3]
                if not counter0 == 0:
                    x0 /= counter0
                    y0 /= counter0
                    dist2 = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, 0, x0, y0, 0)

            if not flag:
                for r in range(len(self.terroristArray)):
                    if self.terroristArray[r] != [] and self.terroristArray[r][0] == 'zephyr2' and 1 < self.terroristArray[r][5] :
                        latest_time2 = self.terroristArray[r][5]
                for s in range(len(self.terroristArray)):
                    if self.terroristArray[s] != [] and self.terroristArray[s][0] == 'zephyr2' and (latest_time2 - self.terroristArray[s][5]) <= 5 :
                        print('zephyr2''den konumu aldim.')
                        counter2 += 1
                        x2 += self.terroristArray[s][2]
                        y2 += self.terroristArray[s][3]
                if not counter2 == 0:
                    x2 /= counter2
                    y2 /= counter2
                    dist3 = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, 0, x2, y2, 0)

            if not flag:
                for r in range(len(self.terroristArray)):
                    if self.terroristArray[r] != [] and self.terroristArray[r][0] == 'zephyr4' and 1 < self.terroristArray[r][5] :
                        latest_time4 = self.terroristArray[r][5]
                for s in range(len(self.terroristArray)):
                    if self.terroristArray[s] != [] and self.terroristArray[s][0] == 'zephyr4' and (latest_time4 - self.terroristArray[s][5]) <= 5 :
                        print('zephyr4''ten konumu aldim.')
                        counter4 += 1
                        x4 += self.terroristArray[s][2]
                        y4 += self.terroristArray[s][3]
                if not counter4 == 0:
                    x4 /= counter4
                    y4 /= counter4
                    dist4 = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, 0, x4, y4, 0)

            if not flag:
                for r in range(len(self.terroristArray)):
                    if self.terroristArray[r] != [] and self.terroristArray[r][0] == 'zephyr5' and 1 < self.terroristArray[r][5] :
                        latest_time5 = self.terroristArray[r][5]
                for s in range(len(self.terroristArray)):
                    if self.terroristArray[s] != [] and self.terroristArray[s][0] == 'zephyr5' and (latest_time5 - self.terroristArray[s][5]) <= 5 :
                        print('zephyr5''ten konumu aldim.')
                        counter5 += 1
                        x5 += self.terroristArray[s][2]
                        y5 += self.terroristArray[s][3]
                if not counter5 == 0:
                    x5 /= counter5
                    y5 /= counter5
                    dist5 = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, 0, x5, y5, 0)

            if dist1 == dist2 and dist2 == dist3 and dist3 == dist4 and dist4 == dist5:
                self.terroristTrackingPoint = [999999, 999999]
            elif dist1 < dist2 and dist1 < dist3 and dist1 < dist4 and dist1 < dist5:
                self.terroristTrackingPoint[0] = x
                self.terroristTrackingPoint[1] = y
            elif dist2 < dist1 and dist2 < dist3 and dist2 < dist4 and dist2 < dist5:
                self.terroristTrackingPoint[0] = x0
                self.terroristTrackingPoint[1] = y0
            elif dist3 < dist1 and dist3 < dist2 and dist3 < dist4 and dist3 < dist5:
                self.terroristTrackingPoint[0] = x2
                self.terroristTrackingPoint[1] = y2
            elif dist4 < dist1 and dist4 < dist2 and dist4 < dist3 and dist4 < dist5:
                self.terroristTrackingPoint[0] = x4
                self.terroristTrackingPoint[1] = y4
            elif dist5 < dist1 and dist5 < dist2 and dist5 < dist3 and dist5 < dist4:
                self.terroristTrackingPoint[0] = x5
                self.terroristTrackingPoint[1] = y5
            #print('hedefi yazdiriyorum.')
            #print(self.terroristTrackingPoint)




