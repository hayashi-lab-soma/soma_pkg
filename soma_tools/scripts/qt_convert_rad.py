#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import rosparam
import sys
import math
from std_msgs.msg import Float64
from maxon_epos_msgs.msg import MotorStates
from maxon_epos_msgs.msg import MotorState

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

app = QApplication([])
window = QWidget()
window.setWindowTitle('Click Sample')
layout = QVBoxLayout()
button = QPushButton('Publish')
steer_edit = QLineEdit('')
rear_edit = QLineEdit('')
front_edit = QLineEdit('')
throttle_edit = QLineEdit('')

layout.addWidget(steer_edit)
layout.addWidget(rear_edit)
layout.addWidget(front_edit)
layout.addWidget(throttle_edit)
layout.addWidget(button)
window.setLayout(layout)

def converter():
    rospy.init_node('conoverter', anonymous=True)   
    pub = rospy.Publisher('radian', MotorStates, queue_size=1)

    steer_rad = math.radians(float(steer_edit.text()))
    rear_rad = math.radians(float(rear_edit.text()))
    front_rad = math.radians(float(front_edit.text()))
    throttle_rad = math.radians(float(throttle_edit.text()))

    motor_cmd = MotorStates()
    motor_cmd.states.append(MotorState(position=steer_rad))
    motor_cmd.states.append(MotorState(position=rear_rad))
    motor_cmd.states.append(MotorState(position=front_rad))
    motor_cmd.states.append(MotorState(position=throttle_rad))


    motor_cmd.header.stamp = rospy.Time.now()
    r = rospy.Rate(1)
    rospy.loginfo("published successfully")
    pub.publish(motor_cmd)
    r.sleep()

def main():
    window.show()

    def on_button_clicked():
        converter()
    button.clicked.connect(on_button_clicked)
    app.exec_()

if __name__ == '__main__':
    main()
