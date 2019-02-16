from math import degrees, atan2, sqrt, pow
import roslib
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from test_ucusu.test_ucusu_scenario_manager import *
from test_ucusu.test_ucusu_traffic_manager import *
from test_ucusu.test_ucusu_battery_manager import *
from test_ucusu.test_ucusu_score_manager import *
from test_ucusu.test_ucusu_sensor_manager import *
from test_ucusu.test_ucusu_mapper import *
from test_ucusu.test_ucusu_pid import *
from test_ucusu.test_ucusu_detector import *
from test_ucusu.test_ucusu_communication import *


def init_node():
    roslib.load_manifest('test_ucusu')
    rospy.init_node('test_ucusu', anonymous=True)
    uav_name = rospy.get_param('~name')
    uav_index = int(uav_name.replace("zephyr", ""))
    return uav_name, uav_index


again = not rospy.is_shutdown()
rate = None


class UAV:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, throttle=0, yaw=0,
                 air_speed=0, ground_speed=0, dist_home=0, cruise_altitude=50,
                 dist_target=0, yaw_target=0, reached_target=False, mode="IDLE",
                 next_way_point=0, total_way_points=0, travelled_distance=0,
                 average_battery_usage=0, battery_capacity=0, battery_remaining=0,
                 heart_beat=0, name="", comm_distance=9999999999, next_landing=0, next_track_point=0,
                 reverse_way_points=False, reverse_landing=False, reverse_track_points=False, buildings={}, help_to="None"):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.throttle = throttle
        self.yaw = yaw
        self.air_speed = air_speed
        self.ground_speed = ground_speed
        self.cruise_altitude = cruise_altitude
        self.dist_home = dist_home
        self.dist_target = dist_target
        self.yaw_target = yaw_target
        self.reached_target = reached_target
        self.mode = mode
        self.next_way_point = next_way_point
        self.total_way_points = total_way_points
        self.travelled_distance = travelled_distance
        self.average_battery_usage = average_battery_usage
        self.battery_capacity = battery_capacity
        self.battery_remaining = battery_remaining
        self.heart_beat = heart_beat
        self.name = name
        self.comm_distance = comm_distance
        self.next_landing = next_landing
        self.next_track_point = next_track_point
        self.reverse_way_points = reverse_way_points
        self.reverse_landing = reverse_landing
        self.reverse_track_points = reverse_track_points
        self.buildings = buildings
        self.help_to = help_to


uav = UAV()


def frequency():
    global rate
    rate.sleep()


class POINT:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


def merge_two_dicts(x, y):
    z = x.copy()   # start with x's keys and values
    z.update(y)    # modifies z with y's keys and values & returns None
    return z


uav_pos_pre = POINT()
uav_pos_now = POINT()
uav_home = POINT()

mission_target = POINT()

uav_time_pre = 0
uav_time_now = 0

pose_sub = None
control_pub = None


def get_pose(pose):
    global uav
    uav.x = pose.position.x
    uav.y = pose.position.y
    uav.z = pose.position.z


def init_uav(uav_name):
    global pose_sub, control_pub, rate
    pose_sub = rospy.Subscriber(uav_name + '_pose', Pose, get_pose)
    control_pub = rospy.Publisher(uav_name + '_control', Twist, queue_size=1)
    rate = rospy.Rate(20)


def distance(x1, y1, z1, x2, y2, z2):
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2))


def angle_between(x1, y1, x2, y2):
    dot = x1 * x2 + y1 * y2  # dot product between [x1, y1] and [x2, y2]
    det = x1 * y2 - y1 * x2  # determinant
    angle = degrees(atan2(det, dot))  # atan2(y, x) or atan2(sin, cos)
    if angle < 0:
        angle = 360 - abs(angle)
    return angle


def update_magnetometer(uav):
    global uav_pos_pre, uav_pos_now
    x1 = uav_pos_now.x - uav_pos_pre.x
    y1 = uav_pos_now.y - uav_pos_pre.y
    x2 = 0
    y2 = 1
    mag = angle_between(x1, y1, x2, y2)
    uav.yaw = mag
    return mag


# Will be implemented as PID controller
# Now it is just a P controller
def to_value(max_value, value, step_size):
    if value < max_value - step_size:
        value = value + step_size
    elif value > max_value + step_size:
        value = value - step_size
    return value


# Update previous time and position of UAV
def update_uav_pre(uav):
    global uav_pos_pre
    uav_pos_pre.x = uav.x
    uav_pos_pre.y = uav.y
    uav_pos_pre.z = uav.z

    global uav_time_pre
    uav_time_pre = rospy.get_time()


# Update current time, position and distance to home of UAV
def update_uav_now(uav):
    # Update current position
    global uav_pos_now
    uav_pos_now.x = uav.x
    uav_pos_now.y = uav.y
    uav_pos_now.z = uav.z

    # Update distance to home
    global uav_home
    uav.dist_home = distance(uav.x, uav.y, uav.z, uav_home.x, uav_home.y, uav_home.z)

    # Update current time
    global uav_time_now
    uav_time_now = rospy.get_time()
    uav.heart_beat = uav_time_now

    # Update travelled distance
    global uav_pos_pre
    uav.travelled_distance += distance(uav_pos_now.x, uav_pos_now.y, 0, uav_pos_pre.x, uav_pos_pre.y, 0)

    # Update average battery usage per meter
    if uav.travelled_distance > 10:
        uav.average_battery_usage = (uav.battery_capacity - uav.battery_remaining) / uav.travelled_distance


# Will be individual uav's and control publishers
# def actuator(uav, control_pub):
def actuator(uav):
    global control_pub
    control = Twist()
    control.linear.x = uav.throttle
    control.angular.x = uav.roll
    control.angular.y = uav.pitch
    control_pub.publish(control)


def set_uav(uav, throttle=450):
    uav.throttle = throttle
    actuator(uav)


def disarm(uav):
    uav.throttle = 0.0
    uav.roll = 0.0
    uav.pitch = 0.0
    actuator(uav)


def set_home(uav):
    global uav_home
    uav_home.x = uav.x
    uav_home.y = uav.y
    uav_home.z = uav.z

    update_uav_pre(uav)
    update_uav_now(uav)


def update_speeds(uav):
    global uav_time_pre, uav_time_now
    global uav_pos_pre, uav_pos_now

    time_diff = uav_time_now - uav_time_pre
    if time_diff > 0:
        ground_diff = distance(uav_pos_now.x, uav_pos_now.y, 0, uav_pos_pre.x, uav_pos_pre.y, 0)
        air_diff = distance(uav_pos_now.x, uav_pos_now.y, uav_pos_now.z, uav_pos_pre.x, uav_pos_pre.y, uav_pos_pre.z)
        ground_speed = ground_diff / time_diff
        air_speed = air_diff / time_diff

        uav.ground_speed = ground_speed
        uav.air_speed = air_speed


pid_pitch = PID(0.10, 0.40, 0.0)  # PID(0.5, 0.0, 0.0)

pid_pitch.SetPoint = 0.0
pid_pitch.setSampleTime(0.01)  # (0.05)

pitch_feedback = 0
pitch_i = 0
pitch_j = 0


def altitude_hold(uav, altitude=50):
    global pid_pitch, pitch_feedback, pitch_i, pitch_j
    altitude_error = 1

    # Throttle up
    set_uav(uav, 600)

    # Stabilize altitude
    pid_pitch.update(uav.pitch)
    output = pid_pitch.output
    if abs(altitude - uav.z) > altitude_error:
        if uav.z < altitude:
            uav.pitch = output
            actuator(uav)
            if pid_pitch.SetPoint < 0:
                pitch_feedback = uav.pitch
            if pitch_i > 9:
                pid_pitch.SetPoint = -0.80
            pitch_j = 0
            pitch_i = pitch_i + 1
        else:
            uav.pitch = output
            actuator(uav)
            if pid_pitch.SetPoint > 0:
                pitch_feedback = uav.pitch
            if pitch_j > 9:
                pid_pitch.SetPoint = 0.80
            pitch_i = 0
            pitch_j = pitch_j + 1
    else:
        uav.pitch = output
        actuator(uav)
        pitch_feedback = uav.pitch
        pid_pitch.SetPoint = 0.0


# not working perfect
def loiter(uav, altitude=50, error=2.5, max_roll=0.5):
    if altitude + error > uav.z > altitude - error:
        uav.pitch = to_value(0, uav.pitch, 0.05)
        uav.roll = to_value(max_roll, uav.pitch, 0.05)
    elif uav.z < altitude - error:
        uav.pitch = to_value(-0.3, uav.pitch, 0.05)  # i am still at low altitude so climb
        # uav.roll = to_value(0, uav.roll, 0.005)
    elif uav.z > altitude + error:
        uav.pitch = to_value(0.3, uav.pitch, 0.05)  # i am at high altitude so dive
        # uav.roll = to_value(0, uav.roll, 0.005)

    # Calculate target distance
    global mission_target
    uav.dist_target = distance(uav.x, uav.y, 0, mission_target.x, mission_target.y, 0)

    # We are getting away from target
    if uav.dist_target > 20:
        uav.reached_target = False
        uav.mode = "GUIDED"


# pid_roll = PID(0.80, 0.90, 0.0001) # img 4
# pid_roll = PID(0.80, 0.95, 0.0005)  # img 5 en iyisi
pid_roll = PID(0.80, 0.95, 0.0005)

pid_roll.SetPoint = 0.0
pid_roll.setSampleTime(0.01)

roll_feedback = 0
roll_i = 0
roll_j = 0


def guided(uav, way_points, altitude, throttle, distance_error=50):
    # Variables those works clean
    yaw_error = 5

    # Use guided as GUIDED
    if uav.mode == GUIDED:
        # If we are in reverse order
        if uav.reverse_way_points:
            way_points.reverse()
        # Get go to point
        x = way_points[uav.next_way_point][0]
        y = way_points[uav.next_way_point][1]

    # Use guide as TRACK
    elif uav.mode == TRACK:
        # If we are in reverse order
        if uav.reverse_track_points:
            way_points.reverse()
        x = way_points[uav.next_track_point][0]
        y = way_points[uav.next_track_point][1]

    # Used guide as LAND
    elif uav.mode == LANDING:
        x = way_points[uav.next_landing][0]
        y = way_points[uav.next_landing][1]

    # Control throttle
    set_uav(uav, throttle)
    # altitude_hold(uav, altitude, altitude_error, max_pitch)
    altitude_hold(uav, altitude)

    # Update mission target
    global mission_target
    mission_target.x = x
    mission_target.y = y
    mission_target.z = altitude

    # Calculate target distance
    uav.dist_target = distance(uav.x, uav.y, 0, mission_target.x, mission_target.y, 0)

    # Calculate target heading
    global uav_pos_pre, uav_pos_now
    x1 = x - uav_pos_pre.x  # now.x ???
    y1 = y - uav_pos_pre.y  # nox.y ???
    x2 = uav_pos_now.x - uav_pos_pre.x
    y2 = uav_pos_now.y - uav_pos_pre.y
    yaw_target = angle_between(x1, y1, x2, y2)
    uav.yaw_target = yaw_target

    # Control heading
    if not uav.reached_target:
        if uav.dist_target > distance_error:
            global pid_roll, roll_feedback, roll_i, roll_j
            pid_roll.update(uav.roll)
            output = pid_roll.output
            if yaw_error < uav.yaw_target < 360 - yaw_error:
                # Slown down
                set_uav(uav, throttle / 2)
                if uav.yaw_target < 180:
                    uav.roll = output
                    temp = uav.pitch
                    uav.pitch -= 0.10
                    actuator(uav)
                    uav.pitch = temp
                    actuator(uav)
                    if pid_roll.SetPoint > 0:
                        roll_feedback = uav.roll
                    if roll_i > 9:
                        pid_roll.SetPoint = -0.40
                    roll_j = 0
                    roll_i = roll_i + 1
                else:
                    uav.roll = output
                    temp = uav.pitch
                    uav.pitch -= 0.10
                    actuator(uav)
                    uav.pitch = temp
                    actuator(uav)
                    if pid_roll.SetPoint > 0:
                        roll_feedback = uav.roll
                    if roll_j > 9:
                        pid_roll.SetPoint = 0.40
                    roll_i = 0
                    roll_j = roll_j + 1
            else:
                # Speed up
                set_uav(uav, throttle)
                uav.roll = output
                actuator(uav)
                roll_feedback = uav.roll
                pid_roll.SetPoint = 0.0

            # We did not reach the target
            return False
        else:
            # If guided is used for way point navigation
            if uav.mode == GUIDED:
                # Go to next point
                uav.next_way_point += 1
                # If next point index is out of range
                if uav.next_way_point == len(way_points):
                    # Change reverse order
                    uav.reverse_way_points = not uav.reverse_way_points
                    # We need to start from the beginning
                    uav.next_way_point = 0

            # If guided is used for tracking
            elif uav.mode == TRACK:
                # Go to next point
                uav.next_track_point += 1
                # If next point index is out of range
                if uav.next_track_point == len(way_points):
                    uav.reverse_track_points = not uav.reverse_track_points
                    # We need to start from the beginning
                    uav.next_track_point = 0

            # If guided is used for landing
            elif uav.mode == LANDING:
                uav.next_landing += 1

            # We reached the target
            return True


def killuav(uav):
    uav.throttle = 0.0
    uav.roll = 0.0
    uav.pitch = -0.50
    actuator(uav)


def land(uav, landing_start, landing_end, distance_error):
    # Create landing sequence
    sequence = [[2*landing_start.x - landing_end.x, 2*landing_start.y - landing_end.y],
                [landing_start.x, landing_start.y],
                [landing_end.x, landing_end.y]]

    # Descending start altitude
    landing_start_altitude = 30  # landing_start.z

    if uav.next_landing == 0:
        # Go rapid to the first landing point
        guided(uav, sequence, landing_start_altitude, 600, distance_error)
        # Boost descending a bit
        if uav.z > landing_start_altitude + 10:
            uav.pitch = 0.40
            actuator(uav)

    elif uav.next_landing == 1:
        # Slow down to second point
        guided(uav, sequence, landing_start_altitude, 300, distance_error)

    elif uav.next_landing == 2:
        # Go to last point with zero altitude, throttle and distance
        guided(uav, sequence, 0, 0, 0.1)
        # Do aggressive maneuver to descend
        if uav.dist_target < 100:
            uav.pitch = 0.60
            actuator(uav)

    # UAV reached landing end
    if uav.z < 0.50:
        killuav(uav)
        uav.mode = LANDED
        uav.next_landing = 0
        return True
    return False
