#!/usr/bin/env python
import rospy
import os
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import message_filters
from tf.transformations import euler_from_quaternion
from math import atan2, pi, cos, sin, sqrt
from datetime import datetime
import matplotlib.pyplot as plt


DIR = os.path.dirname(__file__)
MOTION_FILE_NAME = DIR + '/../../data/motion.txt'


class MotionNoiseRecorder:
    def __init__(self):
        # self.last_real_x = None
        # self.last_real_y = None
        # self.last_real_theta = None

        # self.last_odom_x = None
        # self.last_odom_y = None
        # self.last_odom_theta = None

        # self.last_time = 0
        self.interval_time = 0.1

        self.real_xs = [0]
        self.real_x_offset = None
        self.real_ys = [0]
        self.real_y_offset = None
        self.real_thetas = []
        self.odom_xs = [0]
        self.odom_x_offset = None
        self.odom_ys = [0]
        self.odom_y_offset = None
        self.odom_thetas = [0]

        return

    def callback(self, real, odom):
        # def callback_(self, odom):
        num = 400
        if len(self.odom_xs) >= num:
            return
        print(len(self.odom_xs))
        # new_time = rospy.get_time()
        # if new_time - self.last_time < self.interval_time:
        # return

        # soma_index = 17
        # real_x = real.pose[soma_index].position.x
        # real_y = real.pose[soma_index].position.y
        # real_theta = euler_from_quaternion([real.pose[soma_index].orientation.x, real.pose[soma_index].orientation.y,
        # real.pose[soma_index].orientation.z, real.pose[soma_index].orientation.w])[2]
        real_x = real.pose.pose.position.x
        real_y = real.pose.pose.position.y
        # real_theta = euler_from_quaternion([real.pose.orientation.x, real.pose.orientation.y,
        # real.pose.orientation.z, real.pose.orientation.w])[2]

        front_right_velocity = odom.twist.twist.linear.x

        if front_right_velocity != 0:
            tan_steer_angle = odom.twist.twist.angular.z * 1.04 / front_right_velocity
            # tan_steer_angle = 0

            if tan_steer_angle > 0:
                odom_theta = self.odom_thetas[-1] + front_right_velocity * \
                    self.interval_time / \
                    sqrt((1.04/tan_steer_angle + 0.7/2)**2 + 1.04**2)
            elif tan_steer_angle < 0:
                odom_theta = self.odom_thetas[-1] + front_right_velocity * \
                    self.interval_time / \
                    sqrt((1.04/tan_steer_angle - 0.7/2)**2 + 1.04**2)
            else:
                odom_theta = self.odom_thetas[-1]
            if tan_steer_angle != 0:
                odom_x = self.odom_xs[-1] + 1.04/tan_steer_angle * \
                    (sin(odom_theta) - sin(self.odom_thetas[-1]))
                odom_y = self.odom_ys[-1] - 1.04/tan_steer_angle * \
                    (cos(odom_theta) - cos(self.odom_thetas[-1]))
            else:
                odom_x = self.odom_xs[-1] + front_right_velocity * \
                    self.interval_time * cos(odom_theta)
                odom_y = self.odom_ys[-1] + front_right_velocity * \
                    self.interval_time * sin(odom_theta)
        else:
            odom_x = self.odom_xs[-1]
            odom_y = self.odom_ys[-1]
            odom_theta = self.odom_thetas[-1]

        # odom_x = odom.pose.pose.position.x
        # odom_y = odom.pose.pose.position.y
        # odom_theta = euler_from_quaternion(
            # [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[2]
        # soma_index = 17
        # odom_x = odom.pose[soma_index].position.x
        # odom_y = odom.pose[soma_index].position.y
        # odom_theta = euler_from_quaternion([odom.pose[soma_index].orientation.x, odom.pose[soma_index].orientation.y,
        # odom.pose[soma_index].orientation.z, odom.pose[soma_index].orientation.w])[2]

        if self.odom_x_offset == None:
            self.real_x_offset = real_x
            self.real_y_offset = real_y
            self.odom_x_offset = odom_x
            self.odom_y_offset = odom_y

        real_x -= self.real_x_offset
        real_y -= self.real_y_offset
        odom_x -= self.odom_x_offset
        odom_y -= self.odom_y_offset

        self.real_xs.append(real_x)
        self.real_ys.append(real_y)
        # self.real_thetas.append(real_theta)
        self.odom_xs.append(odom_x)
        self.odom_ys.append(odom_y)
        self.odom_thetas.append(odom_theta)

        # if self.last_real_x != None:
        # real_rot1 = atan2(real_y-self.last_real_y, real_x -
        #   self.last_real_x) - self.last_real_theta
        # real_trans = (real_x-self.last_real_x)**2 + \
        # (real_y-self.last_real_y)**2
        # if real_trans < 1e-10:
        # return
        # if real_rot1 <= -pi/2:
        # real_trans *= -1
        # real_rot1 += pi
        # elif real_rot1 > pi/2:
        # real_trans *= -1
        # real_rot1 -= pi
        # real_rot2 = real_theta - self.last_real_theta - real_rot1

        # odom_rot1 = atan2(odom_y-self.last_odom_y, odom_x -
        #   self.last_odom_x) - self.last_odom_theta
        # odom_trans = (odom_x-self.last_odom_x)**2 + \
        # (odom_y-self.last_odom_y)**2
        # if odom_rot1 <= -pi/2:
        # odom_trans *= -1
        # odom_rot1 += pi
        # elif odom_rot1 > pi/2:
        # odom_trans *= -1
        # odom_rot1 -= pi
        # odom_rot2 = odom_theta - self.last_odom_theta - odom_rot1

        # rot1_error = odom_rot1 - real_rot1
        # if rot1_error > pi/2:
        # rot1_error -= 2*pi
        # elif rot1_error < -pi/2:
        # rot1_error += 2*pi
        # trans_error = odom_trans - real_trans
        # rot2_error = odom_rot2 - real_rot2
        # if rot2_error > pi/2:
        # rot2_error -= 2*pi
        # elif rot2_error < -pi/2:
        # rot2_error += 2*pi

        # tmp = format(real_rot1, '.1E') + " " + \
        # format(real_trans, '.1E') + " " + \
        # format(real_rot2, '.1E') + " "
        # tmp += format(odom_rot1, '.1E') + " " + \
        # format(odom_trans, '.1E') + " " + \
        # format(odom_rot2, '.1E') + " "
        # tmp += format(rot1_error, '.1E') + " " + \
        # format(trans_error, '.1E') + " " + \
        # format(rot2_error, '.1E') + "\n"

        # fo = open(MOTION_FILE_NAME, "a")
        # fo.write(tmp)
        # fo.close()

        # self.last_real_x = real_x
        # self.last_real_y = real_y
        # self.last_real_theta = real_theta

        # self.last_odom_x = odom_x
        # self.last_odom_y = odom_y
        # self.last_odom_theta = odom_theta

        # self.last_time = new_time

        if len(self.odom_xs) == num:
            print("Display")
            plt.plot(self.real_xs, self.real_ys, 'b.')
            # for i in range(len(self.real_xs)):
            # plt.plot([self.real_xs[i], self.real_xs[i]+3e-2*cos(self.real_thetas[i])],
            #  [self.real_ys[i], self.real_ys[i]+3e-2*sin(self.real_thetas[i])], 'b')
            plt.plot(self.odom_xs, self.odom_ys, 'r.')
            for i in range(len(self.odom_xs)):
                plt.plot([self.odom_xs[i], self.odom_xs[i]+3e-2*cos(self.odom_thetas[i])],
                         [self.odom_ys[i], self.odom_ys[i]+3e-2*sin(self.odom_thetas[i])], 'r')

            plt.show()

        return


if __name__ == '__main__':
    rospy.init_node('motion_noise_estimation', anonymous=True)

    node = MotionNoiseRecorder()

    # real_sub = message_filters.Subscriber(
    # '/gazebo/model_states', ModelStates)
    real_sub = message_filters.Subscriber(
        '/odometry/gps', Odometry)
    # odom_sub = message_filters.Subscriber(
    # '/ackermann_steering_controller/odom', Odometry)
    odom_sub = message_filters.Subscriber(
        '/soma/wheel_odom', Odometry)

    sync = message_filters.ApproximateTimeSynchronizer(
        [real_sub, odom_sub], 10, 0.1, allow_headerless=True)
    sync.registerCallback(node.callback)

    # rospy.Subscriber('/soma/wheel_odom', Odometry,
    #  callback=node.callback_, queue_size=1)
    # rospy.Subscriber('/ackermann_steering_controller/odom', Odometry,
    #  callback=node.callback_, queue_size=1)
    # rospy.Subscriber('/gazebo/model_states', ModelStates,
    #  callback=node.callback_, queue_size=1)
    # rospy.Subscriber('/odometry/filtered/local', Odometry,
    #  callback=node.callback_, queue_size=1)
    # rospy.Subscriber('/odometry/gps', Odometry,
    #  callback=node.callback_, queue_size=1)

    rospy.spin()
