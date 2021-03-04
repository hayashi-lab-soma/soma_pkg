#!/usr/bin/env python
import rosbag
import roslib
import rospy
from sensor_msgs.msg import NavSatFix
import csv

BAG_FILE_PATH = '/home/hayashi/Documents/2020-08-09-treepositions/move_2020-08-09-14-01-02.bag'
OUTPUT_FILE_PATH = '/home/hayashi/Documents/2020-08-09-treepositions/move_gps.csv'

def toDMS(degrees):
    deg = int(degrees)
    tmp_f = degrees - deg

    tmp_f = tmp_f*60.0
    min = int(tmp_f)

    tmp_f = tmp_f-min
    sec = tmp_f*60.0
    
    dms = deg*10000 + min*100 + sec
    return dms


def callback(msg):
    lat = msg.latitude
    lon = msg.longitude

    t_sec = msg.header.stamp.secs
    t_n = msg.header.stamp.nsecs

    # print("{},{} -> {},{}".format(lat, lon, toDMS(lat), toDMS(lon)))
    print("{} : {}, {}".format(t_sec+t_n/1000000000.0, toDMS(lat), toDMS(lon)))

    with open('output.csv', 'a') as f:
        writer = csv.writer(f)
        writer.writerow([t_sec+t_n/1000000000.0, toDMS(lat), toDMS(lon)])

if __name__ == '__main__':
    bag = rosbag.Bag(BAG_FILE_PATH)

    for topic,msg,t in bag.read_messages(topics=['/fix']):
        lat = msg.latitude
        lon = msg.longitude
        ts = t.secs + t.nsecs/1000000000
        print("{}:{},{}".format(ts, toDMS(lat), toDMS(lon)))

        with open(OUTPUT_FILE_PATH, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([ts, lat, lon, toDMS(lat), toDMS(lon)])

    bag.close()

#   rospy.init_node('nmea_msgs_converter')
#   rospy.Subscriber("/fix", NavSatFix, callback)
#   rospy.spin()
