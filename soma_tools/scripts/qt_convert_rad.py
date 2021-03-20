#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import rosparam
import sys
import math
from std_msgs.msg import Float32

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

app = QApplication([])
window = QWidget()
window.setWindowTitle('Click Sample')
layout = QVBoxLayout()
button = QPushButton('Publish')
edit = QLineEdit('')
label = QLabel('')

layout.addWidget(edit)
layout.addWidget(label)
layout.addWidget(button)
window.setLayout(layout)

def converter():
    deg = float(edit.text())
    rad = math.radians(deg)

    rospy.init_node('conoverter', anonymous=True)   
    pub = rospy.Publisher('radian', Float32, queue_size=1)

    r = rospy.Rate(1)
    rospy.loginfo(rad)
    pub.publish(rad)

def main():
    window.show()

    def on_button_clicked():
        converter()
    button.clicked.connect(on_button_clicked)

    def on_edit_change(value):
        label.setText(value)
    edit.textChanged.connect(on_edit_change)
    app.exec_()

if __name__ == '__main__':
    main()
