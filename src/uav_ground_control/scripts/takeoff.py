#!/usr/bin/env python
import roslib; roslib.load_manifest('uav_ground_control')
import rospy
import tf
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

takeoff_height = 20
first_pose = None
last_pose = None
speedx = 1
speedy = 1
iris0_control_pub = None
zephyr0_control_pub = None
loiter_is_on = False

def sign(value):
    if value > 0:
        return 1.0
    else:
        return -1.0
        
def clamp(value, max_value):
    if abs(value) > max_value:
        return max_value * sign(value)
    else:
        return value
        
def zephyr0_pose_callback(pose):
    global zephyr0_control_pub, loiter_is_on
    control_msg = Twist()
    control_msg.linear.x = 450
    if pose.position.z < takeoff_height:
        control_msg.angular.y = -0.1
    else:
        control_msg.angular.y = 0.0
    
    if pose.position.z > takeoff_height or loiter_is_on:
        loiter_is_on = True
        control_msg.angular.x = 0.2
    
    zephyr0_control_pub.publish(control_msg)
    

def iris0_pose_callback(pose):
    global first_pose, iris0_pose_sub, takeoff_height, last_pose, speedx, speedy
    
    #print('pose:' + str(pose.position.x) + ' ' + str(pose.position.y) + ' ' + str(pose.position.z))
    quaternion_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    euler_ori = tf.transformations.euler_from_quaternion(quaternion_list)
    #print('ori:' + str(euler_ori[0]) + ' ' + str(euler_ori[1]) + ' ' + str(euler_ori[2]))
    base_throttle = 430
    gain = 10
    err = takeoff_height - pose.position.z
    control_msg = Twist()
    control_msg.linear.z = clamp(base_throttle + err*gain, 450)
    if first_pose is not None:
        xoffset = first_pose.position.x - pose.position.x
        yoffset = first_pose.position.y - pose.position.y
        #print('xoffset:' + str(xoffset) + ' yoffset:' + str(yoffset) + ' speedx:' + str(speedx) + ' speedy:' + str(speedy))
        if abs(err) < 10:
            if abs(xoffset) < 20 and abs(speedx) > 0.001:
                xoffset = speedx*1000
            if abs(yoffset) < 20 and abs(speedy) > 0.001:
                yoffset = speedy*1000
                
            control_msg.angular.x = clamp(-yoffset/1000.0, 0.01)
            control_msg.angular.y = clamp(xoffset/1000.0, 0.01)

    if first_pose is None:
        first_pose = pose
        first_pose.position.x = 50
        first_pose.position.y = 50
    
    if last_pose is not None:
        speedx = last_pose.position.x - pose.position.x
        speedy = last_pose.position.y - pose.position.y
    last_pose = pose
    
    iris0_control_pub.publish(control_msg)
    

if __name__=="__main__":
    rospy.init_node('uav_takeoff_control', anonymous=True)
    zephyr0_pose_sub = rospy.Subscriber('zephyr0_pose', Pose, zephyr0_pose_callback)
    zephyr0_control_pub = rospy.Publisher('zephyr0_control', Twist, queue_size=1)
    
    iris0_pose_sub = rospy.Subscriber('iris0_pose', Pose, iris0_pose_callback)
    iris0_control_pub = rospy.Publisher('iris0_control', Twist, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        
        
        
        
