import time
from geometry_msgs.msg import Twist, Pose
from hadsafha.util import clamp, distance, to_euler, normalize_angle
import math
import rospy


class IrisController:
    def __init__(self, uav_name, air_manager):
        self.air_manager = air_manager
        # self.step_frequency = 100
        # self.last_time = self.get_current_time()
        self.last_uav_pose = Pose()
        # self.reach_threshold = 10
        self.landing_start_pose = None
        self.landing_end_pose = None

        self.landing_state = 'MOVE_TO_LANDING_START'

        self.last_x_error = 0
        self.last_y_error = 0
        self.last_height_error = 0

        self.sum_x_error = 0
        self.sum_y_error = 0
        self.sum_height_error = 0

        self.x_speed = 0
        self.y_speed = 0
        self.height_speed = 0

        self.control_pub = rospy.Publisher(uav_name + '_control', Twist, queue_size=1)
        self.pose_pub = rospy.Subscriber(uav_name + '_pose', Pose, self.pose_callback)

    def pose_callback(self, pose):
        self.x_speed = pose.position.x - self.last_uav_pose.position.x
        self.y_speed = pose.position.y - self.last_uav_pose.position.y
        # print('x_speed:' + str(self.x_speed) + ' y_speed:' + str(self.y_speed))
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
            twist_cmd.linear.z = 460
            twist_cmd.angular.x = 0
            twist_cmd.angular.y = 0
            twist_cmd.angular.z = math.pi
            self.control_pub.publish(twist_cmd)
            # checking whether takeoff completed
            if self.last_uav_pose.position.z >= height:
                return True
            return False
        else:
            return False

    def goto_position(self, x, y, z, throttle, threshold=10):
        dist = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y, \
                        self.last_uav_pose.position.z, x, y, z)
        if dist < threshold:
            return True

        # euler = to_euler(self.last_uav_pose.orientation)
        # ori = euler[2]
        # print('target.x:' + str(x) + ' y:' + str(y) + ' z:' + str(z))
        print('current.x:' + str(self.last_uav_pose.position.x) + ' y:' + \
              str(self.last_uav_pose.position.y) + ' z:' + str(self.last_uav_pose.position.z))
        # print('current ori:' + str(ori))
        # target_ori = normalize_angle(math.atan2(y - self.last_uav_pose.position.y, \
        #                                         x - self.last_uav_pose.position.x) - (math.pi/2))
        # print('target_ori:' + str(target_ori))
        # ori_error = normalize_angle(target_ori - ori)
        # print('error_ori:' + str(ori_error))
        #
        # twist_cmd.angular.x = clamp(ori_error, -0.5, 0.5)
        print('target.x:' + str(x) + ' y:' + str(y) + ' z:' + str(z))

        twist_cmd = Twist()
        x_error = x - self.last_uav_pose.position.x
        y_error = y - self.last_uav_pose.position.y

        if abs(self.x_speed) > 0.05:
            x_error = 0

        if abs(self.y_speed) > 0.05:
            y_error = 0

        twist_cmd.angular.y = clamp(-x_error/10, -0.05, 0.05)
        twist_cmd.angular.x = clamp(y_error/10, -0.05, 0.05)

        twist_cmd.angular.z = math.pi

        height_error = z - self.last_uav_pose.position.z
        twist_cmd.linear.z =  throttle + clamp(height_error*10, -20, 20)

        self.sum_x_error += x_error
        self.sum_y_error += y_error
        self.sum_height_error += height_error

        d_x_error = x_error - self.last_x_error
        d_y_error = y_error - self.last_y_error
        d_height_error = height_error - self.last_height_error

        self.last_x_error = x_error
        self.last_y_error = y_error
        self.last_height_error = height_error

        print('x_error:' + str(x_error) + ' y_error:' + str(y_error) + ' height_error:' + str(height_error))
        self.control_pub.publish(twist_cmd)
        return False

    def land(self):
        completed = self.air_manager.landing_request()
        if completed:
            if self.landing_start_pose is None or self.landing_end_pose is None:
                self.landing_start_pose, self.landing_end_pose = self.air_manager.get_landing_pose()

            print('uav controller landing request completed state:' + self.landing_state)

            if self.landing_state == 'MOVE_TO_LANDING_START':
                goto_start_completed = self.goto_position(self.landing_start_pose[0], \
                                                          self.landing_start_pose[1], \
                                                          self.landing_start_pose[2], 420, 50)
                if goto_start_completed:
                    self.landing_state = 'MOVE_TO_DESCEND'
            elif self.landing_state == 'MOVE_TO_DESCEND':
                # dist_error = distance(self.last_uav_pose.position.x, self.last_uav_pose.position.y,\
                #                       self.last_uav_pose.position.z, self.landing_end_pose[0],\
                #                       self.landing_end_pose[1], self.landing_end_pose[2])
                # dist_error -= 10
                # throttle = clamp(dist_error, 100, 400)
                # print('throttle:' + str(throttle))
                # if self.last_uav_pose.position.z == 0:
                #     throttle = dist_error
                # twist_cmd = Twist()
                # twist_cmd.linear.x = throttle
                # twist_cmd.angular.x = 0
                # twist_cmd.angular.y = 0
                # self.control_publish.publish(twist_cmd)
                # if self.last_uav_pose.position.z == 0:
                #     self.landing_state = 'MOVE_TO_LANDING_START'
                #     print('landing completed')
                #     return True
                goto_landing_completed = self.goto_position(self.landing_end_pose[0],\
                                                            self.landing_end_pose[1],\
                                                            self.landing_end_pose[2] + 5,420)
                if goto_landing_completed:
                    self.landing_state = 'DESCEND'
            elif self.landing_state == 'DESCEND':
                goto_descend_completed = self.goto_position(self.landing_end_pose[0], \
                                                            self.landing_end_pose[1], \
                                                            self.landing_end_pose[2], 420, threshold=5)
                if goto_descend_completed:
                    # make sure that uav is very close to the floor
                    if self.last_uav_pose.position.z < 0.2:
                        # set the landing start state as the first state
                        self.landing_state = 'MOVE_TO_LANDING_START'
                        return True
        else:
            print('waiting the air traffic to land')
            self.goto_position(self.last_uav_pose.position.x, self.last_uav_pose.position.y,\
                               self.last_uav_pose.position.z, 430, threshold=0.0)

        return False

    def stop_motors(self):
        twist_cmd = Twist()
        # no throttle
        twist_cmd.linear.z = 0
        self.control_pub.publish(twist_cmd)

    def get_latest_pose(self):
        return self.last_uav_pose

    # @staticmethod
    # def get_current_time():
    #     # returns time in milliseconds
    #     return time.time()*1000
