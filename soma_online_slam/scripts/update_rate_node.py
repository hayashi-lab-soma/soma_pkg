#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2


class UpdateRateNode:
    def __init__(self):
        self.pointcloud = PointCloud2()

        self.rate = 1
        self.start = 0

        return

    def update_pointcloud(self, data):
        # if rospy.get_time() - self.start >= 1.0/self.rate * 0.95:
        if True:
            # print("POINTCLOUD UPDATE")

            self.pointcloud.data = data.data
            self.pointcloud.fields = data.fields
            self.pointcloud.header = data.header
            self.pointcloud.height = data.height
            self.pointcloud.is_bigendian = data.is_bigendian
            self.pointcloud.is_dense = data.is_dense
            self.pointcloud.point_step = data.point_step
            self.pointcloud.row_step = data.row_step
            self.pointcloud.width = data.width

        return


if __name__ == '__main__':
    rospy.init_node('update_rate', anonymous=True)

    node = UpdateRateNode()

    rate = rospy.Rate(node.rate)

    rospy.Subscriber('/velodyne_points', PointCloud2,
                     callback=node.update_pointcloud, queue_size=1)
    pointcloud_pub = rospy.Publisher(
        '/lidar_pointcloud', PointCloud2, queue_size=1)

    while not rospy.is_shutdown():
        node.start = rospy.get_time()
        pointcloud_pub.publish(node.pointcloud)
        rate.sleep()
