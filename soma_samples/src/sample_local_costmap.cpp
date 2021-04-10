#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_costmap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string costmap_ns = pnh.param<std::string>("costmap_ns", "costmap");

  tf2_ros::Buffer *tfBuf = new tf2_ros::Buffer();
  tf2_ros::TransformListener *tfListener = new tf2_ros::TransformListener(*tfBuf);

  costmap_2d::Costmap2DROS costmap(costmap_ns, *tfBuf);
  costmap.resetLayers();
  costmap.start();

  ros::spin();
  return 0;
}