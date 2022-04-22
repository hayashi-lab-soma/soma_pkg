#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose
from jsk_recognition_msgs.msg import Int32Stamped


# Update number of clusters (trees)
def cluster(data):
    global cluster_num
    cluster_num = data.data
    print("\nNew clusters received (" + str(cluster_num) + ")...")

    return


# If all trees centers updated, publish them
def loop():
    global cluster_num, updates, pose

    update = True
    for i in range(cluster_num):
        update = update and updates[i]
    if update:
        pub.publish(trees_centers)
        print("Trees positions updated")

        for i in range(cluster_num):
            updates[i] = False
            trees_centers.poses[i] = pose

    return


# Generic function to register trees centers update
def fuse_centers(index, time, pose):
    trees_centers.header.stamp = time
    trees_centers.poses[index] = pose

    global updates
    updates[index] = True
    loop()

    return


# Specific functions for each tree

def fuse_centers0(data):
    fuse_centers(0, data.header.stamp, data.poses[0])

    return


def fuse_centers1(data):
    fuse_centers(1, data.header.stamp, data.poses[0])

    return


def fuse_centers2(data):
    fuse_centers(2, data.header.stamp, data.poses[0])

    return


def fuse_centers3(data):
    fuse_centers(3, data.header.stamp, data.poses[0])

    return


def fuse_centers4(data):
    fuse_centers(4, data.header.stamp, data.poses[0])

    return


def fuse_centers5(data):
    fuse_centers(5, data.header.stamp, data.poses[0])

    return


def fuse_centers6(data):
    fuse_centers(6, data.header.stamp, data.poses[0])

    return


def fuse_centers7(data):
    fuse_centers(7, data.header.stamp, data.poses[0])

    return


def fuse_centers8(data):
    fuse_centers(8, data.header.stamp, data.poses[0])

    return


def fuse_centers9(data):
    fuse_centers(9, data.header.stamp, data.poses[0])

    return


def fuse_centers10(data):
    fuse_centers(10, data.header.stamp, data.poses[0])

    return


def fuse_centers11(data):
    fuse_centers(11, data.header.stamp, data.poses[0])

    return


def fuse_centers12(data):
    fuse_centers(12, data.header.stamp, data.poses[0])

    return


def fuse_centers13(data):
    fuse_centers(13, data.header.stamp, data.poses[0])

    return


def fuse_centers14(data):
    fuse_centers(14, data.header.stamp, data.poses[0])

    return


def fuse_centers15(data):
    fuse_centers(15, data.header.stamp, data.poses[0])

    return


if __name__ == '__main__':
    cluster_num = 0
    updates = 16*[False]

    trees_centers = PoseArray()
    trees_centers.header.frame_id = "velodyne"
    trees_centers.poses = []
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    for i in range(16):
        trees_centers.poses.append(pose)

    rospy.init_node('tree_coordinates', anonymous=True)

    rospy.Subscriber('/clustering/cluster_num', Int32Stamped,
                     callback=cluster, queue_size=1)

    rospy.Subscriber('/tree0', PoseArray,
                     callback=fuse_centers0, queue_size=1)
    rospy.Subscriber('/tree1', PoseArray,
                     callback=fuse_centers1, queue_size=1)
    rospy.Subscriber('/tree2', PoseArray,
                     callback=fuse_centers2, queue_size=1)
    rospy.Subscriber('/tree3', PoseArray,
                     callback=fuse_centers3, queue_size=1)
    rospy.Subscriber('/tree4', PoseArray,
                     callback=fuse_centers4, queue_size=1)
    rospy.Subscriber('/tree5', PoseArray,
                     callback=fuse_centers5, queue_size=1)
    rospy.Subscriber('/tree6', PoseArray,
                     callback=fuse_centers6, queue_size=1)
    rospy.Subscriber('/tree7', PoseArray,
                     callback=fuse_centers7, queue_size=1)
    rospy.Subscriber('/tree8', PoseArray,
                     callback=fuse_centers8, queue_size=1)
    rospy.Subscriber('/tree9', PoseArray,
                     callback=fuse_centers9, queue_size=1)
    rospy.Subscriber('/tree10', PoseArray,
                     callback=fuse_centers10, queue_size=1)
    rospy.Subscriber('/tree11', PoseArray,
                     callback=fuse_centers11, queue_size=1)
    rospy.Subscriber('/tree12', PoseArray,
                     callback=fuse_centers12, queue_size=1)
    rospy.Subscriber('/tree13', PoseArray,
                     callback=fuse_centers13, queue_size=1)
    rospy.Subscriber('/tree14', PoseArray,
                     callback=fuse_centers14, queue_size=1)
    rospy.Subscriber('/tree15', PoseArray,
                     callback=fuse_centers15, queue_size=1)

    pub = rospy.Publisher("/trees_centers", PoseArray, queue_size=1)

    rospy.spin()
