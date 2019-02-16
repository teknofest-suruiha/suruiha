import time
from geometry_msgs.msg import Twist, Pose
from iztech_onair.util import clamp, distance, to_euler, normalize_angle
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
        self.landing_start_pose2 = None
        self.landing_end_pose2 = None

        self.landing_state = 'GOTO_LANDING'

        self.last_x_error = 0
        self.last_y_error = 0
        self.last_height_error = 0

        self.sum_x_error = 0
        self.sum_y_error = 0
        self.sum_height_error = 0

        self.x_speed = 0
        self.y_speed = 0
        self.height_speed = 0

        self.k=1

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
            twist_cmd.linear.z = 480
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

        if self.last_uav_pose.position.z < 2:
            self.takeoff(35)

        if dist < 50:
            throttle = 430

        if dist < 20:
            throttle = 425

        print('current.x:' + str(self.last_uav_pose.position.x) + ' y:' + \
              str(self.last_uav_pose.position.y) + ' z:' + str(self.last_uav_pose.position.z))
        print('target.x:' + str(x) + ' y:' + str(y) + ' z:' + str(z))

        twist_cmd = Twist()

        x_error = ((x - self.last_uav_pose.position.x)*10/dist)
        y_error = ((y - self.last_uav_pose.position.y)*10/dist)
        height_error = z - self.last_uav_pose.position.z


        if abs(self.x_speed) > 0.20 and abs(self.y_speed) > 0.20:
            x_error = 0
            y_error = 0

        elif abs(self.x_speed) > 0.20:
            x_error = 0

        elif abs(self.y_speed) > 0.20:
            y_error = 0


        twist_cmd.angular.x = clamp(y_error, -0.20, 0.20)
        twist_cmd.angular.y = clamp(-x_error, -0.20, 0.20)
        twist_cmd.angular.z = math.pi

        throttle = throttle + clamp(height_error*100, -10, 10)

        if dist < 20 and height_error < 2:
            throttle = 440

        twist_cmd.linear.z = throttle

        self.sum_x_error += x_error
        self.sum_y_error += y_error
        self.sum_height_error += height_error

        self.last_x_error = x_error
        self.last_y_error = y_error
        self.last_height_error = height_error

        #print('x_error:' + str(x_error) + ' y_error:' + str(y_error) + ' height_error:' + str(height_error))
        self.control_pub.publish(twist_cmd)
        return False

    def land(self):
        if self.landing_state == 'GOTO_LANDING':
            if self.landing_start_pose2 is None or self.landing_end_pose2 is None:
                self.landing_start_pose2, self.landing_end_pose2 = self.air_manager.get_landing_pose()
            print('uav goes to landing start')
            goto_start_completed = self.goto_position(self.landing_start_pose2[0]+5,
                                                      self.landing_start_pose2[1]+5,
                                                      self.landing_start_pose2[2], 420)
            if goto_start_completed:
                self.landing_state = 'MOVE_TO_LANDING_START'
        else:
            completed = self.air_manager.landing_request()
            if completed:
                if self.landing_start_pose is None or self.landing_end_pose is None:
                    self.landing_start_pose, self.landing_end_pose = self.air_manager.get_landing_pose()

                print('uav controller landing request completed state:' + self.landing_state)

                if self.landing_state == 'MOVE_TO_LANDING_START':
                    goto_start_completed2 = self.goto_position(self.landing_start_pose[0],
                                                              self.landing_start_pose[1],
                                                              self.landing_start_pose[2], 420, 50)
                    if goto_start_completed2:
                        self.landing_state = 'MOVE_TO_DESCEND'
                elif self.landing_state == 'MOVE_TO_DESCEND':
                    goto_landing_completed = self.goto_position(self.landing_end_pose[0],
                                                                self.landing_end_pose[1],
                                                                self.landing_end_pose[2] + 5, 400)
                    if goto_landing_completed:
                        self.landing_state = 'DESCEND'
                elif self.landing_state == 'DESCEND':
                    goto_descend_completed = self.goto_position(self.landing_end_pose[0],
                                                                self.landing_end_pose[1],
                                                                self.landing_end_pose[2], 400, threshold=5)
                    if goto_descend_completed:
                        # make sure that uav is very close to the floor
                        if self.last_uav_pose.position.z < 0.2:
                            # set the landing start state as the first state
                            self.landing_state = 'MOVE_TO_LANDING_START'
                            return True
            else:
                print('waiting the air traffic to land')
                self.goto_position(self.last_uav_pose.position.x, self.last_uav_pose.position.y,
                                   self.last_uav_pose.position.z, 440, threshold=5.0)

            return False
        return False

    def stop_motors(self):
        twist_cmd = Twist()
        # no throttle
        twist_cmd.linear.z = 0
        self.control_pub.publish(twist_cmd)

    def loiter(self, x, y, z):

        throttle = 440

        twist_cmd = Twist()

        x_error = (x - self.last_uav_pose.position.x)*10
        y_error = (y - self.last_uav_pose.position.y)*10
        height_error = z - self.last_uav_pose.position.z


        if abs(self.x_speed) > 0.20 and abs(self.y_speed) > 0.20:
            x_error = 0
            y_error = 0

        elif abs(self.x_speed) > 0.20:
            x_error = 0

        elif abs(self.y_speed) > 0.20:
            y_error = 0


        twist_cmd.angular.x = clamp(y_error, -0.20, 0.20)
        twist_cmd.angular.y = clamp(-x_error, -0.20, 0.20)
        twist_cmd.angular.z = math.pi

        throttle = throttle + clamp(height_error*100, -20, 20)

        twist_cmd.linear.z = throttle

        self.sum_x_error += x_error
        self.sum_y_error += y_error
        self.sum_height_error += height_error

        self.last_x_error = x_error
        self.last_y_error = y_error
        self.last_height_error = height_error

        self.control_pub.publish(twist_cmd)
        if self.k == 5000:
            self.k=1
            return True

    def get_latest_pose(self):
        return self.last_uav_pose

    # @staticmethod
    # def get_current_time():
    #     # returns time in milliseconds
    #     return time.time()*1000
