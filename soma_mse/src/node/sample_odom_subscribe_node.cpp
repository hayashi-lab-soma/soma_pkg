#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::TransformListener *tfListener;
tf2_ros::Buffer *tfBuf;

void odom_callback(const nav_msgs::OdometryConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_odom_subscriber_node");
  ros::NodeHandle nh;

  tfBuf = new tf2_ros::Buffer();
  tfListener = new tf2_ros::TransformListener(*tfBuf);

  ros::Subscriber odom_sub = nh.subscribe("/odom", 3, odom_callback);

  ros::spin();
  return 0;
}

void odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
  ROS_INFO("%s, %.2f,%.2f",
           msg->header.frame_id.c_str(),
           msg->pose.pose.position.x,
           msg->pose.pose.position.y);

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tfBuf->lookupTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(1000));
  }
  catch (tf2::LookupException &e)
  {
    ROS_WARN("%s",e.what());
    return;
  }

  //transform robot pose to "map" coordinate
  geometry_msgs::PoseStamped _odom, out;
  _odom.header = msg->header;
  _odom.pose = msg->pose.pose;
  tf2::doTransform(_odom, out, transform);

  ROS_INFO("%s, %.2f,%.2f",
           out.header.frame_id.c_str(),
           out.pose.position.x,
           out.pose.position.y);

  return;
}
