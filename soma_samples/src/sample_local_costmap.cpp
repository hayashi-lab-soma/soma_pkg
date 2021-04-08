#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_costmap");
  ros::NodeHandle nh;

  tf2_ros::Buffer *tfBuf = new tf2_ros::Buffer();
  tf2_ros::TransformListener *tfListener() = new tf2_ros::TransformListener(*tfBuf);
  costmap_2d::Costmap2DROS costmap("local_costmap_rgbd_front", *tfBuf);
  costmap.start();
  ros::spin();
  return 0;
}