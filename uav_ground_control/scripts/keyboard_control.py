#!/usr/bin/env python
import roslib; roslib.load_manifest('uav_ground_control')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
	      
throttle = 0.0
pitch = 0.0
roll = 0.0



def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('uav_keyboard_control', anonymous=True)
	pose_topic_name = rospy.get_param('~pose')
	control_topic_name = rospy.get_param('~control')
	pub = rospy.Publisher(control_topic_name, Twist, queue_size = 1)
	

	try:
		while not rospy.is_shutdown():
			print 'throttle up a down d | pitch up s down w | roll up q down e | to exit t'
			key = getKey()
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
   			control_cmd.linear.z = throttle # for iris quadrotor
   			control_cmd.linear.x = throttle # for zephyr fixeed wing
			control_cmd.angular.y = pitch
			control_cmd.angular.x = roll
			pub.publish(control_cmd)

	except:
		print e

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
