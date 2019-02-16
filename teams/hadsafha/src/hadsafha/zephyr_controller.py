import time
# from uav_sample_controller.air_traffic_manager import TAKEOFF, ONAIR
from geometry_msgs.msg import Twist, Pose
from hadsafha.util import clamp, distance, to_euler, normalize_angle
import math
import rospy


class ZephyrController:
    def __init__(self, uav_name, air_manager, uav_wakeup_time):
        self.air_manager = air_manager
        # self.step_frequency = 100
        # self.last_time = self.get_current_time()
        self.last_uav_pose = Pose()
        # self.reach_threshold = 10
        self.landing_start_pose = None
        self.landing_end_pose = None

        self.landing_state = 'MOVE_TO_LANDING_START'

        self.uav_wakeup_time = uav_wakeup_time
        self.uav_priority = 0

        self.control_pub = rospy.Publisher(uav_name + '_control', Twist, queue_size=1)
        self.pose_pub = rospy.Subscriber(uav_name + '_pose', Pose, self.pose_callback)

    def get_uav_priority(self):
        return self.uav_priority

    def pose_callback(self, pose):
        self.last_uav_pose = pose

    # def step(self):
    #     elapsed_time = self.get_current_time()-self.last_time
    #     if elapsed_time > self.step_frequency:
    #         self.last_time = self.get_current_time()
    #         if self.air_manager.get_status() == TAKEOFF:
    #             # takeoff to given height
    #             takeoff_completed = self.takeoff(100)
    #             if takeoff_completed:
    #                 self.air_manager.set_status(ONAIR)
    #         elif self.air_manager.get_status() == ONAIR:
    #             self.loiter(20, 100)

    def takeoff(self, height):
        completed = self.air_manager.takeoff_request()
        # now it is time to takeoff and be onair
        if completed:
            print('takeoff request is completed')
            # print('last.height:' + str(self.last_uav_pose.position.z))
            twist_cmd = Twist()
            # checking whether takeoff completed
            if self.last_uav_pose.position.z >= height:
                twist_cmd.linear.x = 400
                twist_cmd.angular.x = 0
                twist_cmd.angular.y = 0
                self.control_pub.publish(twist_cmd)
                return True

            if self.last_uav_pose.position.z < 1:
                twist_cmd.linear.x = 450
                twist_cmd.angular.x = 0
                twist_cmd.angular.y = 0
            else:
                twist_cmd.linear.x = 450
                twist_cmd.angular.x = 0
                twist_cmd.angular.y = -0.3
            self.control_pub.publish(twist_cmd)
            return False
        else:
            return False

    def goto_position(self, x, y, z, throttle, threshold=10):
        dist = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, \
                        self.last_uav_pose.position.z, x, y, z)
        if dist < threshold:
            return True

        twist_cmd = Twist()
        twist_cmd.linear.x = throttle
        euler = to_euler(self.last_uav_pose.orientation)
        ori = euler[2]
        print('target.x:' + str(x) + ' y:' + str(y) + ' z:' + str(z))
        print('current.x:' + str(self.last_uav_pose.position.x) + ' y:' + \
              str(self.last_uav_pose.position.y) + ' z:' + str(self.last_uav_pose.position.z))
        print('current ori:' + str(ori))
        target_ori = normalize_angle(math.atan2(y - self.last_uav_pose.position.y, \
                                                x - self.last_uav_pose.position.x) - (math.pi/2))
        print('target_ori:' + str(target_ori))
        ori_error = normalize_angle(target_ori - ori)
        print('error_ori:' + str(ori_error))

        twist_cmd.angular.x = clamp(ori_error, -0.5, 0.5)

        height_error = z - self.last_uav_pose.position.z
        twist_cmd.angular.y =  clamp(-height_error/10, -0.4, 0.4)
        self.control_pub.publish(twist_cmd)
        return False

    def loiter(self, radius, height):
        twist_cmd = Twist()
        twist_cmd.linear.x = 400
        height_error = height - self.last_uav_pose.position.z
        twist_cmd.angular.y = -height_error/10
        twist_cmd.angular.x = 0.2
        self.control_pub.publish(twist_cmd)

    def land(self):
        completed = self.air_manager.landing_request()
        if completed:
            print('uav controller landing request completed')
            if self.landing_start_pose is None or self.landing_end_pose is None:
                self.landing_start_pose, self.landing_end_pose = self.air_manager.get_landing_pose()

            if self.landing_state == 'MOVE_TO_LANDING_START':
                goto_start_completed = self.goto_position(self.landing_start_pose[0], \
                                                          self.landing_start_pose[1], \
                                                          self.landing_start_pose[2], 400, 50)
                if goto_start_completed:
                    self.landing_state = 'DESCENDING'
            elif self.landing_state == 'DESCENDING':
                dist_error = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y,\
                                      self.last_uav_pose.position.z, self.landing_end_pose[0],\
                                      self.landing_end_pose[1], self.landing_end_pose[2])
                dist_error -= 10
                throttle = clamp(dist_error, 100, 400)
                print('throttle:' + str(throttle))
                # if self.last_uav_pose.position.z == 0:
                #     throttle = dist_error
                # twist_cmd = Twist()
                # twist_cmd.linear.x = throttle
                # twist_cmd.angular.x = 0
                # twist_cmd.angular.y = 0
                # self.control_pub.publish(twist_cmd)
                # if self.last_uav_pose.position.z == 0:
                #     self.landing_state = 'MOVE_TO_LANDING_START'
                #     print('landing completed')
                #     return True
                goto_landing_completed = self.goto_position(self.landing_end_pose[0],\
                                                            self.landing_end_pose[1],\
                                                            self.landing_end_pose[2],throttle)
                if goto_landing_completed:
                    self.landing_state = 'MOVE_TO_LANDING_START'
                    return True
        else:
            self.loiter(20, 50)

        return False

    def stop_motors(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0
        twist_cmd.angular.x = 0
        twist_cmd.angular.y = 0
        self.control_pub.publish(twist_cmd)

    def get_latest_pose(self):
        return self.last_uav_pose

    # @staticmethod
    # def get_current_time():
    #     # returns time in milliseconds
    #     return time.time()*1000
