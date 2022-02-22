#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from scipy.stats import multivariate_normal
import sys


def add_motor_noise(data):
    command = [data.linear.x, data.angular.z]

    v, omega = command
    noise = list(sys.argv[1].replace("]", "[").split("["))

    v_noise = list(noise[2].split(","))
    omega_noise = list(noise[4].split(","))
    v_noise = [float(f) for f in v_noise]
    omega_noise = [float(f) for f in omega_noise]

    v_sigma = v_noise[0]*abs(v) + v_noise[1]*abs(omega) + v_noise[2]
    omega_sigma = omega_noise[0] * \
        abs(v) + omega_noise[1]*abs(omega) + omega_noise[2]
    sigma = [[v_sigma, 0],
             [0, omega_sigma]]

    noisy_command = multivariate_normal.rvs(command, sigma)[:]
    noisy_v, noisy_omega = noisy_command

    msg_pub = Twist()
    msg_pub.linear.x, msg_pub.linear.y, msg_pub.linear.z = noisy_v, 0.0, 0.0
    msg_pub.angular.x, msg_pub.angular.y, msg_pub.angular.z = 0.0, 0.0, noisy_omega

    cmd_vel_pub.publish(msg_pub)

    return


if __name__ == '__main__':
    rospy.init_node('motor_noise', anonymous=True)

    rospy.Subscriber('/ackermann_steering_controller/command',
                     Twist, callback=add_motor_noise, queue_size=1)

    cmd_vel_pub = rospy.Publisher('/ackermann_steering_controller/cmd_vel',
                                  Twist, queue_size=1)

    rospy.spin()
