import numpy
from datetime import datetime, date
from geometry_msgs.msg import Pose

class Map:
    def __init__(self, width, height, this_name):
        # Bulunan objeler ve konumlari
        self.detected_objects = {}
        self.uav_name = this_name
        self.terrorist_detected = None
        self.width = width
        self.height = height

        # self.map_matrix = numpy.zeros(shape=(width, height))

    def add_object(self, name, pose, time=None, uav_name=None):
        if uav_name is None:
            uav_name = self.uav_name
        if time is None:
            time = datetime.now().time()

        obj = {"pose": pose,
               "time": time,
               "uav_name": uav_name,
               "is_first_time_seen": 0}

        if name not in self.detected_objects:
            self.detected_objects[name] = []
            # eger terorist tespit edilmis ve bunu suanki iha yapmissa takip degerini ata
            if self.terrorist_detected is None and "terrorist" in name:
                if uav_name is self.uav_name:
                    self.terrorist_detected = name

        # Onceden bu obje eklenmemisse
        if not self.is_object_existing_before(obj, name):
            # Eger bina daha onceden tespit edilmisse tekrar listeye ekleme

            if "building" in name:
                if len(self.detected_objects[name]) is not 0:
                    return -1
            # Eger terorist son uc saniye icinde eklenmisse rotasini listeye ekleme
            if "terrorist" in name:
                if uav_name is self.uav_name:
                    if len(self.detected_objects[name]) is not 0:
                        this_obj_time = self.get_newest_detected_object(name)["time"]
                        difference = datetime.combine(date.today(), this_obj_time) - datetime.combine(date.today(), time)
                        if difference.seconds < 40000:
                            return -1
                    else:
                        obj["is_first_time_seen"] = 1

        # obje verilerini performans icin sadelestir
        self.simplify_object(name)
        self.detected_objects[name].append(obj)


    def is_object_existing_before(self, obj, name):
        if obj in self.detected_objects[name]:
            return True
        return False

    def simplify_object(self, name):
        '''
        Verilen objenin son 10 durumunu saklar geri kalan posizyonlarini siler
        :param name:
        :return:
        '''


        del self.detected_objects[name][0:-10]

        #print("SIMPLIFIED["+str(name)+"]")
        #print(self.detected_objects[name])


    def get_newest_detected_object(self, name, index=None):
        try:
            if index is None:
                index = -1
            # Sort by date
            self.detected_objects[name] = sorted(
                self.detected_objects[name],
                key=lambda x: x["time"],
                reverse=False)

            #print(self.detected_objects[name])

            return self.detected_objects[name][index]
        except IndexError:
            return None

    def get_object(self, name):

        if name in self.detected_objects:
            return self.detected_objects[name]
        else:
            return -1

    def get_all_detected_objects(self):
        return self.detected_objects

    def sync_map(self, teammate_detected_objects):
        for teammate_name in teammate_detected_objects:
            for object in teammate_detected_objects[teammate_name]:
                # verinin kaynagi suanki iha degilse
                if object["uav_name"] != self.uav_name:
                    obj_name = str(object["object_type"])
                    detected_time = datetime.strptime(object["time"], '%H:%M:%S.%f').time()
                    detector_uav_name = teammate_name
                    pose = Pose()
                    pose.position.x = float(object["x"])
                    pose.position.y = float(object["y"])
                    pose.position.z = float(object["z"])
                    self.add_object(obj_name, pose, detected_time, detector_uav_name)

    def get_objects_json_formatted(self):
        objects = self.detected_objects
        data = []
        for obj in objects:
            for element in objects[obj]:
                pose = element["pose"]
                time = element["time"]
                uav_name = element["uav_name"]
                data.append({
                    "object_type": str(obj),
                    "x": str(pose.position.x),
                    "y": str(pose.position.y),
                    "z": str(pose.position.z),
                    "time": str(time),
                    "uav_name": str(uav_name)
                })
        return data

    def disallow_overflowing_from_axis(self, axis_length, value):
        '''
        Araclarin harita disina cikmasini lojik olarak engeller
        :param axis_length:
        :param value:
        :return:
        '''
        length = axis_length / 2
        if value >= length:
            value = length - 40
        elif value <= (-1) * length:
            value = (-1) * length + 40

        return value
    def reverse_map_to_start_from_nearest_point(self, route, current_pose):
        x = current_pose.position.x
        y = current_pose.position.y
        start_point = route[0]
        end_point = route[-1]

        current_to_start = (x - start_point[0] )**2 + (y - start_point[1])**2
        current_to_end = (x - end_point[0])**2 + (y- end_point[1])**2

        if current_to_start > current_to_end:
            return reversed(route)
        return route

    def square_spiral(self,  center_x, center_y, left_top_x, left_top_y, right_bottom_x, right_bottom_y, fov, uav_height):
        path = []
        width = abs(right_bottom_x - left_top_x)
        height = abs(right_bottom_y - left_top_y)

        division_rate = height / fov

        for i in range(1, division_rate):
            rate_height = i * height / (fov * 2)
            rate_width = i * width / (fov * 2)
            top_left = [center_x - rate_width, center_y + rate_height, uav_height]
            top_right = [center_x + rate_width, center_y + rate_height, uav_height]
            bottom_right = [center_x + rate_width, center_y - rate_height, uav_height]
            bottom_left = [center_x - rate_width, center_y - rate_height, uav_height]

            if center_x - rate_width and center_x + rate_width in range(left_top_x, right_bottom_x):
                if center_y + rate_height and center_y - rate_height in range(left_top_y, right_bottom_y):
                    square = [top_left, top_right, bottom_right, bottom_left]
                    path.extend(square)
        return path



    def slice_area_to_paths(self, x0, y0, x1, y1, height, space):
        paths=[]

        ## Default attir
        x_curr = x0
        y_prev = y0
        factor = 1
        distance = abs(x0 - x1)
        ## Azalan ise
        if x0 - x1 == abs(x0 - x1):
            factor = -1
        step = distance / space

        for i in range(0, step):

            x = self.disallow_overflowing_from_axis(self.width, x_curr)
            y_0 = self.disallow_overflowing_from_axis(self.height, y0)
            y_1 = self.disallow_overflowing_from_axis(self.height, y1)
            if y_prev == y0:
                way_point0 = [x, y_0, height]
                way_point1 = [x, y_1, height]
                y_prev = y1
            elif y_prev == y1:
                way_point0 = [x, y_1, height]
                way_point1 = [x, y_0, height]
                y_prev = y0
            paths.append(way_point0)
            paths.append(way_point1)
            x_curr = x_curr + factor * space

        return paths
