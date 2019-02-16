from suruiha_gazebo_plugins.msg import UAVMessage
import rospy
import pickle
from math import sqrt, pow


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


def distance(x1, y1, z1, x2, y2, z2):
    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2))


def merge_two_dicts(x, y):
    z = x.copy()   # start with x's keys and values
    z.update(y)    # modifies z with y's keys and values & returns None
    return z


class CommunicationManager:
    def __init__(self, uav_name, uav):
        self.uav_name = uav_name
        self.msg_publisher = rospy.Publisher('comm_request', UAVMessage, queue_size=10)
        rospy.Subscriber('comm_' + self.uav_name, UAVMessage, self.message_received)
        self.uav = uav
        self.teammate_poses = {"zephyr0": UAV(), "zephyr1": UAV(), "zephyr2": UAV(), "zephyr3": UAV(), "zephyr4": UAV(),
                               "zephyr5": UAV()}
        self.online_teammates = {}
        self.offline_teammates = {}
        self.dead_teammates = {}
        self.never_teammates = {}
        self.needs_help = "None"

    def message_received(self, msg):
        payload = msg.msg
        sender = msg.sender
        self.teammate_poses[sender] = pickle.loads(payload)

    def publish_pose(self):
        uav = self.uav
        payload = pickle.dumps(uav)
        uav_msg = UAVMessage()
        uav_msg.sender = uav.name
        uav_msg.msg = payload
        self.msg_publisher.publish(uav_msg)

    # UAVs those are healthy
    def get_online_teammates(self):
        # Checks hear_beat and distance
        # If heart_beat received within 2 seconds inside comm_distance
        # If True then comrade UAV is working properly
        uav = self.uav
        self.online_teammates = {k: self.teammate_poses[k] for k, v in self.teammate_poses.items() if
                                 abs(rospy.get_time() - float(v.heart_beat)) < 2 and v.heart_beat != 0 and distance(uav.x, uav.y, uav.z, v.x,
                                                                                              v.y,
                                                                                              v.z) < uav.comm_distance - 10 and k != uav.name}
        # Absolutely do nothing
        return self.online_teammates

    # UAVs those are outside the comm range
    def get_offline_teammates(self):
        # Checks hear_beat and distance
        # If heart_beat not received for 2 seconds near outside comm_distance
        # If True then UAV may still do its work outside comm_distance
        uav = self.uav
        self.offline_teammates = {k: self.teammate_poses[k] for k, v in self.teammate_poses.items() if
                                  abs(rospy.get_time() - float(v.heart_beat)) >= 2 and v.heart_beat != 0 and distance(uav.x, uav.y, uav.z,
                                                                                                v.x, v.y,
                                                                                                v.z) > uav.comm_distance - 10 and k != uav.name}
        # Do nothing because you can not possibly be sure whats happening
        # This is the restriction of this competition
        # Function is just for being aware of other UAVs
        # Future implementations could be done
        return self.offline_teammates

    # UAVs those are started but has internal error
    def get_dead_teammates(self):
        # Checks hear_beat and distance
        # If heart_beat not received for 2 seconds absolute within comm_distance
        # If True then get responsibilities of dead UAV with order
        uav = self.uav
        self.dead_teammates = {k: self.teammate_poses[k] for k, v in self.teammate_poses.items() if
                               abs(rospy.get_time() - float(v.heart_beat)) >= 2 and v.heart_beat != 0 and distance(uav.x, uav.y, uav.z, v.x,
                                                                                             v.y,
                                                                                             v.z) < uav.comm_distance - 10 and k != uav.name}
        return self.dead_teammates

    # UAVs those never started
    def get_never_teammates(self):
        # Checks only if UAV has 0 heart_beat
        # Never started UAVs heart_beat set to 0
        uav = self.uav
        self.never_teammates = {k: self.teammate_poses[k] for k, v in self.teammate_poses.items() if v.heart_beat == 0 and k != uav.name}
        # Do nothing because never started UAVs will be started
        return self.never_teammates

    def who_needs_help(self):
        # Decide who will help who then help
        # Restrict only one help by setting helper UAV help to the dead team mate
        uav = self.uav

        # Get who wants help
        deads = self.dead_teammates
        trackers = {k: self.teammate_poses[k] for k, v in self.teammate_poses.items() if k != uav.name and v.mode == "TRACK"}

        # Concatenate dead and tracker uavs
        needs_help = merge_two_dicts(deads, trackers)

        # Find UAVs who is helping currently
        helpers = {k: self.online_teammates[k] for k, v in self.online_teammates.items() if
                   k != uav.name and v.mode != "TRACK" and v.help_to.find("zephyr") > -1}

        # If someone helps to stressed UAV it does not need help
        for key in [helpers[name].help_to for name in helpers.keys()]:
            try:
                del needs_help[key]
            except:
                pass

        # Only online, not helper and not tracker uavs can help
        will_helpers = {k: self.online_teammates[k] for k, v in self.online_teammates.items() if
                        k != uav.name and v.mode != "TRACK" and v.help_to == "None"}

        # If we have a stressed
        if len(needs_help.keys()) > 0:
            # We will help to stressed UAVs one by one so get first
            stressed = needs_help[needs_help.keys()[0]]

            # Find my distance to stressed uav
            my_dist = distance(uav.x, uav.y, 0, stressed.x, stressed.y, 0)

            # Find other UAVs distances to stressed UAV
            if will_helpers > {}:
                distances = []
                for key in will_helpers.keys():
                    dist = distance(will_helpers[key].x, will_helpers[key].y, 0, stressed.x, stressed.y, 0)
                    distances.append(dist)
                min_dist = min(distances)
            else:
                min_dist = 9999999999

            # Only help if you are in GUIDED mode and not helping anyone else
            if uav.help_to == "None" and uav.mode != "TRACK" and my_dist <= min_dist:
                who = needs_help.keys()[0]
                return who
            else:
                return "None"
        return "None"
