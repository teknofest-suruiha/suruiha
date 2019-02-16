import time
from geometry_msgs.msg import Twist, Pose
from iztech_onair.util import clamp, distance, to_euler, normalize_angle
import math
import rospy



class ZephyrController:
    def __init__(self, uav_name, air_manager):
        self.air_manager = air_manager
        # self.step_frequency = 100
        # self.last_time = self.get_current_time()
        self.last_uav_pose = Pose()
        # self.reach_threshold = 10
        self.landing_start_pose = None
        self.landing_end_pose = None
        self.landing_start_pose2 = None
        self.landing_end_pose2 = None

        self.landing_state = 'GOTO_LANDING'

        self.k=1
        self.l=0.0001

        self.control_pub = rospy.Publisher(uav_name + '_control', Twist, queue_size=1)
        self.pose_pub = rospy.Subscriber(uav_name + '_pose', Pose, self.pose_callback)

    def pose_callback(self, pose):
        self.last_uav_pose = pose

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
        dist = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y,
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
        #print('current ori:' + str(ori))
        target_ori = normalize_angle(math.atan2(y - self.last_uav_pose.position.y,
                                                x - self.last_uav_pose.position.x) - (math.pi/2))
        #print('target_ori:' + str(target_ori))
        ori_error = normalize_angle(target_ori - ori)
        #print('error_ori:' + str(ori_error))

        twist_cmd.angular.x = clamp(ori_error, -0.5, 0.5)

        height_error = z - self.last_uav_pose.position.z
        twist_cmd.angular.y =  clamp(-height_error/10, -0.4, 0.4)
        self.control_pub.publish(twist_cmd)
        return False

    def loiter(self, radius, height):
        twist_cmd = Twist()
        twist_cmd.linear.x = 600
        height_error = height - self.last_uav_pose.position.z
        twist_cmd.angular.y = -height_error/10
        if self.k == 1:
            self.l = -0.42
        elif self.k == 350:
            self.l = -0.3
        elif self.k == 800:
            self.l = -0.2
        elif self.k == 1400:
            self.l = -0.16
        elif self.k == 2100:
            self.l = -0.12
        elif self.k == 2850:
            self.l = -0.09
        elif self.k == 3500:
            self.k=1
            self.l=0.0005
            return True
        twist_cmd.angular.x = self.l
        self.k = self.k+1
        print(str(self.k))
        print(str(self.l))
        self.control_pub.publish(twist_cmd)

    def loiter_larger(self, radius, height):
        twist_cmd = Twist()
        twist_cmd.linear.x = 550
        height_error = height - self.last_uav_pose.position.z
        twist_cmd.angular.y = -height_error/10
        if self.k == 1:
            self.l = -0.09
        elif self.k == 1000:
            self.l = -0.06
        elif self.k == 2000:
            self.l = -0.03
        elif self.k == 3500:
            self.k=1
            self.l=0.00059
            return True
        twist_cmd.angular.x = self.l
        self.k = self.k+1
        print(str(self.k))
        print(str(self.l))
        self.control_pub.publish(twist_cmd)

    def detection(self, height):
        twist_cmd = Twist()
        twist_cmd.linear.x = 600
        height_error = height - self.last_uav_pose.position.z
        twist_cmd.angular.y = -height_error / 10
        if self.k == 1:
            self.l = -0.42
        elif self.k == 50:
            self.k = 1
            self.l = 0.0005
            return True
        twist_cmd.angular.x = self.l
        self.k = self.k + 1
        print(str(self.k))
        print(str(self.l))
        self.control_pub.publish(twist_cmd)

    def land(self):
        if self.landing_state == 'GOTO_LANDING':
            if self.landing_start_pose2 is None or self.landing_end_pose2 is None:
                self.landing_start_pose2, self.landing_end_pose2 = self.air_manager.get_landing_pose()
                if self.landing_start_pose2[0] == self.landing_end_pose2[0] and self.landing_start_pose2[1] > 0:
                    self.landing_start_pose2[1] = self.landing_start_pose2[1] - 20
                elif self.landing_start_pose2[0] == self.landing_end_pose2[0] and self.landing_start_pose2[1] < 0:
                    self.landing_start_pose2[1] = self.landing_start_pose2[1] + 20
                elif self.landing_start_pose2[1] == self.landing_end_pose2[1] and self.landing_start_pose2[0] > 0:
                    self.landing_start_pose2[0] = self.landing_start_pose2[0] - 20
                elif self.landing_start_pose2[1] == self.landing_end_pose2[1] and self.landing_start_pose2[0] < 0:
                    self.landing_start_pose2[0] = self.landing_start_pose2[0] + 20
            print('uav goes to landing start')
            goto_start_completed = self.goto_position(self.landing_start_pose2[0],
                                                      self.landing_start_pose2[1],
                                                      self.landing_start_pose2[2]+1, 450)
            if goto_start_completed:
                self.landing_state = 'MOVE_TO_LANDING_START'

        else:
            completed = self.air_manager.landing_request()
            if completed:
                print('uav controller landing request completed')
                if self.landing_start_pose is None or self.landing_end_pose is None:
                    self.landing_start_pose, self.landing_end_pose = self.air_manager.get_landing_pose()
                    if self.landing_start_pose[0] == self.landing_end_pose[0] and self.landing_start_pose[1] > 0:
                        self.landing_start_pose[1] = self.landing_start_pose[1] - 20
                    elif self.landing_start_pose[0] == self.landing_end_pose[0] and self.landing_start_pose[1] < 0:
                        self.landing_start_pose[1] = self.landing_start_pose[1] + 20
                    elif self.landing_start_pose[1] == self.landing_end_pose[1] and self.landing_start_pose[0] > 0:
                        self.landing_start_pose[0] = self.landing_start_pose[0] - 20
                    elif self.landing_start_pose[1] == self.landing_end_pose[1] and self.landing_start_pose[0] < 0:
                        self.landing_start_pose[0] = self.landing_start_pose[0] + 20

                elif self.landing_state == 'MOVE_TO_LANDING_START':

                    goto_start_completed2 = self.goto_position(self.landing_start_pose[0],                                         self.landing_start_pose[1],
                                                              self.landing_start_pose[2], 400)
                    if goto_start_completed2:
                        self.landing_state = 'DESCENDING'
                elif self.landing_state == 'DESCENDING':
                    dist_error = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y,
                                          self.last_uav_pose.position.z, self.landing_end_pose[0],
                                          self.landing_end_pose[1], self.landing_end_pose[2])
                    dist_error -= 15
                    throttle = clamp(dist_error, 25, 250)
                    print('throttle:' + str(throttle))

                    goto_landing_completed = self.goto_position(self.landing_end_pose[0],
                                                                self.landing_end_pose[1],
                                                                self.landing_end_pose[2], throttle)
                    if goto_landing_completed:
                        self.landing_state = 'GOTO_LANDING'
                        return True
                else:
                    self.loiter(20, 90)
                    return False
            else:
                self.loiter(20, 90)
                return False
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