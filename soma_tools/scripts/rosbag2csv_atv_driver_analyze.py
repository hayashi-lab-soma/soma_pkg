#!/use/bin/python
# You should to use this script by python2 series
import rosbag
import numpy as np
import os

BAG_FILE='/mnt/d/bagfiles/atv_control_exp/p_0_8_d_0_1_2021-03-23-16-02-35.bag'

if __name__=='__main__':
  print('rosbag to csv')
  print('input bag file:', BAG_FILE)
  bag = rosbag.Bag(BAG_FILE)

  names = []
  data_frame = []
  for topic,msg,t in bag.read_messages():
    if topic == '/maxon_bringup/get_all_states':
      motor_states = msg.states

      #get motor names
      for m in motor_states:
        names.append(m.motor_name)

      #get positions
      col = []
      col.append(t.to_time())
      for m in motor_states:
        col.append(m.position)

      #
      data_frame.append(col)
  
  data_frame = np.array(data_frame)

  np.savetxt('./motor_states.csv',data_frame,delimiter=',')