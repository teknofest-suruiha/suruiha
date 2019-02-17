#!/usr/bin/env python

from test_ucusu.test_ucusu_utils import *  # do not touch

# Create UAV
uav_name, uav_index = init_node()
init_uav(uav_name)
uav.name = uav_name
uav.mode = IDLE
print("UAV: " + str(uav.name) + " created.")

# Create guidance system
battery_manager = BatteryManager(uav_name)
score_manager = ScoreManager()
sensor_manager = SensorManager(uav_name)
terrorist_detector = TerroristDetector(sensor_manager)
comm_manager = CommunicationManager(uav_name, uav)
print("Guidance system for " + str(uav.name) + " created.")

# Get scenario paramaters
num_uavs, comm_distance, area_size, runway, uavs = get_scenario_params()
uav.comm_distance = comm_distance
print("Got flight data from scenario")
mapper_area_size = [area_size["width"], area_size["height"]]
mapper_left_bottom = [runway["left_bottom"]["x"], runway["left_bottom"]["y"]]
mapper_right_top = [runway["right_top"]["x"], runway["right_top"]["y"]]
my_uav = ["zephyr", uav_name.split("zephyr")[1]]
my_uav_configs = [i for i in uavs if i["type"] == my_uav[0] and i["index"] == int(my_uav[1])]
uav.cruise_altitude = float(my_uav_configs[0]["sensor"]["max_height"]) - 10
uav_batteries = [i["battery_capacity"] for i in uavs if i["type"] == "zephyr"]
print("Got scenario parameters.")

# Generate way points
way_point_output = way_point_generator(mapper_area_size, mapper_left_bottom, mapper_right_top,
                                       uav_batteries, num_uavs, uav.cruise_altitude)
way_points = way_point_output[uav_index]
uav.total_way_points = len(way_points)
print("Generated " + str(len(way_points)) + " way points.")

# Get landing pose
landing_start = POINT()
landing_end = POINT()
result = traffic_manager(uav_name, LANDING_POSE)
print(result)
result = [float(x) for x in result.split(" ")]
landing_start.x = result[0]
landing_start.y = result[1]
landing_start.z = result[2]
landing_end.x = result[6]
landing_end.y = result[7]
landing_end.z = result[8]
print("Got landing pose.")

# Handle air traffic issue
print("Requesting air traffic clearance")
result = traffic_manager(uav_name, STATUS)
while result == TAKEN:
    print("Airport is in use")
    time.sleep(1)
    print("Requesting air traffic clearance")
    result = traffic_manager(uav_name, STATUS)
    print(result)
result = traffic_manager(uav_name, TAKEOFF_REQUEST)
print(result)
print("Air traffic clearance complete")
print("Departure")
print("Happy hunts...")

uav.mode = TAKEOFF
print("Mode changed to: " + str(uav.mode))
print("Cruise altitude set to: " + str(uav.cruise_altitude))

# Handle 0 battery issue
uav.battery_remaining = battery_manager.get_remaining_battery()
uav.battery_capacity = battery_manager.get_capacity_battery()

while uav.battery_remaining == 0.0:
    time.sleep(1)
    uav.battery_remaining = battery_manager.get_remaining_battery()
    uav.battery_capacity = battery_manager.get_capacity_battery()

set_home(uav)
landed = False
previous_mode = GUIDED
while again:
    # Update current UAV status
    update_uav_now(uav)

    # Update magnetometer of UAV
    update_magnetometer(uav)

    # Update ground and air speeds of UAV
    update_speeds(uav)

    # Battery is enough to the routine
    if uav.battery_remaining > 0.23*(uav.dist_home + 300):
        pass
    # If battery is not enough go to refuel
    else:
        # Only GUIDED, TRACK and HELP modes can come to LANDING
        if uav.mode != LANDED and uav.mode != IDLE:
            uav.mode = LANDING

    # Take off to cruise height
    if uav.mode == TAKEOFF:
        altitude_hold(uav, uav.cruise_altitude)
        if uav.z >= uav.cruise_altitude:
            uav.mode = previous_mode

    # Go as GUIDED
    if uav.mode == GUIDED:
        previous_mode = GUIDED
        guided(uav, way_points, uav.cruise_altitude, 600, 50)

    # If landing confirmed
    if uav.mode == LANDING:
        result = traffic_manager(uav_name, STATUS)
        # If runway reserved for UAV
        if result != ALLOCATED_TO_LAND:
            # Request landing
            result = traffic_manager(uav_name, LANDING_REQUEST)
        # Do landing operations return True if landed
        landed = land(uav, landing_start, landing_end, 30)

    # We landed do post processing
    if uav.mode == LANDED:
        result = traffic_manager(uav_name, STATUS)
        # If we landed runway will be available
        if result == AVAILABLE:
            # UAV is IDLE
            uav.mode = IDLE
            uav.travelled_distance = 0
            # Disarm UAV
            disarm(uav)
            # Landed is completed reset it
            landed = False

    # Do takeoff process after refuel
    if uav.mode == IDLE:
        result = traffic_manager(uav_name, STATUS)
        # If runway reserved for UAV
        if result == AVAILABLE:
            traffic_manager(uav_name, TAKEOFF_REQUEST)

        # Make sure that battery replaced
        if uav.battery_remaining > uav.battery_capacity - 10:
            uav.mode = TAKEOFF

    # Get air traffic status
    result = traffic_manager(uav_name, STATUS)

    # Get battery status
    uav.battery_remaining = battery_manager.get_remaining_battery()
    uav.battery_capacity = battery_manager.get_capacity_battery()

    # Get score status
    current_score = score_manager.get_current_score()
    area_score = current_score.area_score
    detection_score = current_score.detection_score
    tracking_score = current_score.tracking_score
    total_score = current_score.total_score

    # Get teammates
    teammates_online = comm_manager.get_online_teammates()
    teammates_offline = comm_manager.get_offline_teammates()
    teammates_dead = comm_manager.get_dead_teammates()
    teammates_never = comm_manager.get_never_teammates()
    who_needs_help = comm_manager.who_needs_help()

    # Help to stressed teammates
    if who_needs_help.find("zephyr") > -1 and uav.help_to == "None":
        stressed_uav_index = int(who_needs_help.split("zephyr")[1])
        way_points = way_points + way_point_output[stressed_uav_index]
        uav.total_way_points = len(way_points)
        # Waypoints added only once
        uav.help_to = who_needs_help

    # Get detected things and send teammates_online to detection
    terrorists, buildings, terrorist_building, track_points, track_points_changed = \
        terrorist_detector.detect_subjects(teammates_online)
    # Set buildings to UAV class to share with other UAVs
    uav.buildings = merge_two_dicts(uav.buildings, buildings)

    # Detected activities change flight mode
    if track_points > [] and uav.mode == GUIDED:
        uav.mode = TRACK

    # Detected activities follow it
    if uav.mode == TRACK:
        previous_mode = TRACK
        # Track way points changed start from beginning
        if track_points_changed:
            uav.next_track_point = 0
        # Only one object detected loiter around
        if len(track_points) == 1:
            guided(uav, track_points, uav.cruise_altitude, 600, 0.1)
        # More than one track point detected do normal guided
        elif len(track_points) > 1:
            guided(uav, track_points, uav.cruise_altitude, 600, 40)

    # Print OSD, do not edit unless change in UAV class!
    osd = ""
    osd += "Name: " + str(uav_name) + " Mode: " + str(uav.mode) + " Duty: " + str(previous_mode) + " Help: " + str(uav.help_to) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Time: " + "{0:.2f}".format(uav.heart_beat) + " Air Traffic: " + result + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Total Battery Capacity: " + "{0:.2f}".format(uav.battery_capacity) + "\n"
    osd += "Remaining Battery Capacity: " + "{0:.2f}".format(uav.battery_remaining) + "\n"
    osd += "Average Battery Usage: " + "{0:.2f}".format(uav.average_battery_usage) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Area score: " + "{0:.2f}".format(area_score) + "\n"
    osd += "Detection score: " + "{0:.2f}".format(detection_score) + "\n"
    osd += "Tracking score: " + "{0:.2f}".format(tracking_score) + "\n"
    osd += "Total score: " + "{0:.2f}".format(total_score) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "X = " + "{0:.2f}".format(uav.x) + " Y = " + "{0:.2f}".format(uav.y) + " Z = " + "{0:.2f}".format(uav.z) + "\n"
    osd += "Roll = " + "{0:.2f}".format(uav.roll) + " Pitch = " + "{0:.2f}".format(uav.pitch) + " Throttle = " + "{0:.2f}".format(uav.throttle) + " Azimuth = " + "{0:.2f}".format(uav.yaw) + "\n"
    osd += "Ground Speed = " + "{0:.2f}".format(uav.ground_speed) + " Air Speed = " + "{0:.2f}".format(uav.air_speed) + "\n"
    osd += "Home Distance = " + "{0:.2f}".format(uav.dist_home) + " Total Travelled Distance: " + "{0:.2f}".format(uav.travelled_distance) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Next Way Point Distance = " + "{0:.2f}".format(uav.dist_target) + " Next Way Point Yaw = " + "{0:.2f}".format(uav.yaw_target) + "\n"
    osd += "Next Way Point Number: " + str(uav.next_way_point) + " Total Way Points: " + str(uav.total_way_points) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Detected Terrorists: " + str(len(terrorists)) + "\n"
    osd += "Detected Buildings: " + str(len(buildings)) + "\n"
    osd += "Terrorist Building: " + str(terrorist_building) + "\n"
    osd += "-------------------------------------------------------------" + "\n"
    osd += "Connected: " + ", ".join([element for element in teammates_online.keys()]) + "\n"
    osd += "Out Of Range: " + ", ".join([element for element in teammates_offline.keys()]) + "\n"
    osd += "Out Of Service: " + ", ".join([element for element in teammates_dead.keys()]) + "\n"
    osd += "Never Started: " + ", ".join([element for element in teammates_never.keys()]) + "\n"
    osd += "-------------------------------------------------------------"
    print(osd)

    # Publish latest pose to  other UAVs
    comm_manager.publish_pose()

    # Update UAV previous
    update_uav_pre(uav)  # update uav pre

    # Set ROS rate
    frequency()
