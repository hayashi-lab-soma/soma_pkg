#!/usr/bin/python

# from soma_msgs.srv import GetGraphBasedPlan
# from soma_msgs.srv import GetGraphBasedPlanResponse
from nav_msgs.srv import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy

def get_graph_based_plan(req):
  print('service called')
  trajectory = Path()

  pose = PoseStamped()
  pose.pose.position.x = 1.0
  pose.pose.position.y = 1.0
  pose.pose.position.z = 0.0
  pose.pose.orientation.x = 0.0
  pose.pose.orientation.y = 0.0
  pose.pose.orientation.z = 0.0
  pose.pose.orientation.w = 1.0
  trajectory.poses.append(pose)

  return GetPlanResponse(trajectory)

def bringup_server():
  rospy.init_node('sample_soma_service_server')
  s = rospy.Service('get_graph_based_plan', GetPlan, get_graph_based_plan)
  rospy.spin()

if __name__=='__main__':
  bringup_server()