#!/usr/bin/python
import rospy
import numpy as np
import math
from maxon_epos_msgs.msg import MotorState

TARGET_MOTOR_STATE_MSG = '/maxon_brinup/rear_brake/set_state'
TARGET_POSITION = 0.0
CONST_RPM = 3500

def step_input():
  pub = rospy.Publisher(TARGET_MOTOR_STATE_MSG, MotorState, queue_size=3)
  rospy.init_node('sample_step_input',anonymous=True)
  rate=rospy.Rate(10) #10Hz

  msg = MotorState()
  msg.position = math.radians(TARGET_POSITION)
  msg.velocity = CONST_RPM

  pub.publish(msg) # step input

if __name__ == '__main__':
  try:
    step_input()
  except rospy.ROSInterruptException:
    pass
