import rospy
from suruiha_gazebo_plugins.msg import UAVTracking
from std_msgs.msg import String
import math
from math import pow, sqrt, floor


def merge_two_dicts(x, y):
    z = x.copy()   # start with x's keys and values
    z.update(y)    # modifies z with y's keys and values & returns None
    return z


def distance(x1, y1, z1, x2, y2, z2):
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2))


def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)


def most_common(lst):
    return max(set(lst), key=lst.count)


def fit(X, Y):

    def mean(Xs):
        return sum(Xs) / len(Xs)
    m_X = mean(X)
    m_Y = mean(Y)

    def std(Xs, m):
        normalizer = len(Xs) - 1
        return math.sqrt(sum((pow(x - m, 2) for x in Xs)) / normalizer)

    def pearson_r(Xs, Ys):

        sum_xy = 0
        sum_sq_v_x = 0
        sum_sq_v_y = 0

        for (x, y) in zip(Xs, Ys):
            var_x = x - m_X
            var_y = y - m_Y
            sum_xy += var_x * var_y
            sum_sq_v_x += pow(var_x, 2)
            sum_sq_v_y += pow(var_y, 2)
        return sum_xy / math.sqrt(sum_sq_v_x * sum_sq_v_y)

    r = pearson_r(X, Y)

    b = r * (std(Y, m_Y) / std(X, m_X))
    A = m_Y - b * m_X

    def line(x):
        return b * x + A
    return line


def future_past(predictor, terrorist, time):
    if predictor > {}:
        return [predictor[terrorist][0](time), predictor[terrorist][1](time)]


def predict_building(predictor, buildings):
    if predictor > {}:
        time = int(rospy.get_time())
        detected_building = []
        for t in range(time, 0, -1):
            for j in predictor.keys():
                predicted_point = future_past(predictor, j, t)
                for i in buildings.keys():
                    dist = distance(predicted_point[0], predicted_point[1], 0, buildings[i][0], buildings[i][1], 0)
                    if dist < 50:
                        detected_building.append(i)
                        break
        if detected_building:
            most_common_building = most_common(detected_building)
            counter = detected_building.count(most_common_building)

            # I detected at least 5 terrorist
            if len(predictor.keys()) >= 5:
                # At least more than half of terrorists result the building
                if counter >= int(floor(float(len(detected_building))/float(2)) + 1):
                    # Safely return the building
                    return most_common_building


class TerroristDetector:
    def __init__(self, sensor_manager):
        self.tracking_publisher = rospy.Publisher('/terrorist_tracking', UAVTracking, queue_size=1)
        self.detection_publisher = rospy.Publisher('/terrorist_detection', String, queue_size=1)
        self.sensor_manager = sensor_manager

        # These will be used for processing
        self.buildings = {}
        self.terrorists = {}
        self.terrorist_building = "None"
        self.time = float(rospy.get_time())
        self.future_terrorist_mean = [0, 0]
        self.track_points = []
        self.track_points_changed = False

    def detect_subjects(self, teammates_online):
        perception = self.sensor_manager.get_perception()

        tracking_message = UAVTracking()
        for i in range(len(perception.types)):
            if perception.names[i].find("terrorist") >= 0:
                tracking_message.names.append(perception.names[i])
                tracking_message.poses.append(perception.poses[i])
        if len(tracking_message.names) > 0:
            self.tracking_publisher.publish(tracking_message)
        else:
            # Send the tracking data which is come from fusion (legal?)
            pass

        # Get terrorists and buildings as machine readable
        time_now = float(rospy.get_time())
        if time_now >= self.time + 0.20:
            for i in range(len(perception.types)):
                if perception.names[i].find("terrorist") >= 0:
                    if perception.names[i] not in self.terrorists:
                        self.terrorists[perception.names[i]] = []
                    self.terrorists[perception.names[i]].append(
                        [perception.poses[i].position.x, perception.poses[i].position.y, time_now])

                if perception.names[i].find("building") >= 0:
                    self.buildings[perception.names[i]] = [perception.poses[i].position.x,
                                                           perception.poses[i].position.y]
            self.time = time_now

        predictor = {}

        for i in [i for i in self.terrorists.keys() if len(self.terrorists[i]) > 1]:

            name = i

            # Select x and corresponding time
            select_x = [[i[0], i[2]] for i in self.terrorists[name]]
            # Select y and corresponding time
            select_y = [[i[1], i[2]] for i in self.terrorists[name]]

            # Select x
            x_values = [x[0] for x in select_x]
            # Select time
            t_values = [x[1] for x in select_x]
            # Fit x
            fit_x = fit(t_values, x_values)

            # Select y
            y_values = [y[0] for y in select_y]
            # Select time
            t_values = [y[1] for y in select_y]
            # Fit y
            fit_y = fit(t_values, y_values)

            # Concat x and y as predictor dict
            predictor[name] = [fit_x, fit_y]

        if predictor > {}:
            # Calculate terrorist means
            terrorist_means = {}
            for name in predictor.keys():
                findit = [[i[0], i[1]] for i in self.terrorists[name]]
                xs = [i[0] for i in findit]
                ys = [i[1] for i in findit]
                terrorist_means[name] = [mean(xs), mean(ys)]
            # Find the right most point
            right = max(terrorist_means.values(), key=lambda x: distance(mean([i[0] for i in terrorist_means.values()]),
                                                                         mean([i[1] for i in terrorist_means.values()]), 0,
                                                                         x[0], x[1], 0))
            # Find the middle least point
            middle = min(terrorist_means.values(), key=lambda x: distance(mean([i[0] for i in terrorist_means.values()]),
                                                                          mean([i[1] for i in terrorist_means.values()]), 0,
                                                                          x[0], x[1], 0))
            # Find the left most point
            left = max(terrorist_means.values(), key=lambda x: distance(right[0], right[1], 0, x[0], x[1], 0))

            # Which terrorist is on the right
            right_name = terrorist_means.keys()[terrorist_means.values().index(right)]

            # Which terrorist is on the left
            left_name = terrorist_means.keys()[terrorist_means.values().index(left)]

            # Where will be the left terrorist in x seconds later
            left_terrorist = future_past(predictor, left_name, float(rospy.get_time()) + 15)

            # Where will be the right terrorist in x seconds later
            right_terrorist = future_past(predictor, right_name, float(rospy.get_time()) + 15)

            # Middle point of the left and right
            middle_terrorist = [(left_terrorist[0] + right_terrorist[0])/2, (left_terrorist[1] + right_terrorist[1])/2]

            # Calculate distance between last and current mean of terrorist locations
            dist = distance(middle_terrorist[0], middle_terrorist[1], 0, self.future_terrorist_mean[0], self.future_terrorist_mean[1], 0)

            # If they moved noticeable
            if dist > 50:
                # Change self means
                self.future_terrorist_mean[0] = middle_terrorist[0]
                self.future_terrorist_mean[1] = middle_terrorist[1]

                temp = [left_terrorist, middle_terrorist, right_terrorist]
                if temp == self.track_points:
                    self.track_points_changed = False
                else:
                    self.track_points_changed = True
                    self.track_points = temp
                    # print(temp)

        # Create way points to follow terrorists
        # Change UAV mode to TRACK

        # Do movement fusion and calculate where terrorists started
        # Find the building which is closest to the starting point
        # Send the calculated building below

        # Merge detected to buildings with other UAVs
        for team_mate in teammates_online.keys():
            uav = teammates_online[team_mate]
            uav_buildings = uav.buildings
            self.buildings = merge_two_dicts(self.buildings, uav_buildings)

        # Now you are ready for detecting the building
        # Ensure that you never send the terrorist building before
        if self.terrorist_building == "None":
            if predictor > {}:
                if self.buildings > {}:
                    # Predict the building
                    terrorist_building = predict_building(predictor, self.buildings)

                    # If there is a prediction
                    if terrorist_building:
                        # Send building to judge server
                        detection_message = String()
                        detection_message.data = terrorist_building
                        self.detection_publisher.publish(detection_message)

                        # Set building var to something to send once
                        self.terrorist_building = terrorist_building

        # Return as safe and sound
        return self.terrorists, self.buildings, self.terrorist_building, self.track_points, self.track_points_changed
