#!/usr/bin/env python
import roslib
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from suruiha_gazebo_plugins.srv import AirTraffic
from sample_team.air_traffic_manager import AirTrafficManager
import math


roslib.load_manifest('sample_team')
settings = termios.tcgetattr(sys.stdin)


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    rospy.init_node('keyboard_controller', anonymous=True)
    uav_name = rospy.get_param('~name')

    # connect to air traffic controller
    air_manager = AirTrafficManager(uav_name)

    pose_topic_name = uav_name + '_pose'
    control_topic_name = uav_name + '_control'

    control_pub = rospy.Publisher(control_topic_name, Twist, queue_size=1)

    throttle = 0.0
    pitch = 0.0
    roll = 0.0

    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    state = 'TAKEOFF'


    while not rospy.is_shutdown():
        if state == 'TAKEOFF':
            completed = air_manager.takeoff_request()
            if completed:
                state = 'ONAIR'
        elif state == 'ONAIR':
            print 'throttle up a down d | pitch up s down w | roll up q down e | to exit t'
            key = get_key()
            if key == 'e':
                roll = roll - 0.01
            if key == 'q':
                roll = roll + 0.01
            if key == 'a':
                throttle = throttle + 10
            if key == 'd':
                throttle = throttle - 10
            if key == 'w':
                pitch = pitch - 0.01
            if key == 's':
                pitch = pitch + 0.01
            if key == 't':
                break

            print 'current throttle:', str(throttle), ' pitch:', str(pitch), ' roll:', str(roll)
            control_cmd = Twist()
            control_cmd.linear.z = throttle  # for iris quadrotor
            control_cmd.linear.x = throttle  # for zephyr fixeed wing
            control_cmd.angular.y = pitch
            control_cmd.angular.x = roll
            control_cmd.angular.z = math.pi
            control_pub.publish(control_cmd)
        else:
            print('unknown state:' + state)

        ros_rate.sleep()
