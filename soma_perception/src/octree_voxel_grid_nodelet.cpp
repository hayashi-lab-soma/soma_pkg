#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

namespace soma_perception
{
  class OctreeVoxelGridNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;

    OctreeVoxelGridNodelet() {}
    virtual ~OctreeVoxelGridNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      input_points_sub = nh.subscribe("input_points",
                                      3,
                                      &OctreeVoxelGridNodelet::cloud_callback,
                                      this);

      output_points_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_octree", 3);

      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      resolution = pnh.param<float>("resolution", 0.01);
    }

  private:
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
      //if point cloud empty
      if (input->data.empty())
      {
        return;
      }
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
      pcl::fromROSMsg(*input, *cloud);
      pcl::PointCloud<PointT>::Ptr cloud_voxeled (new pcl::PointCloud<PointT> ());

      // generate octree
      pcl::octree::OctreePointCloud<PointT> octree(resolution);
      // add point cloud to octree
      octree.setInputCloud(cloud);
      octree.addPointsFromInputCloud();
      // get points where grid is occupied
      pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector point_vec;
      octree.getOccupiedVoxelCenters(point_vec);
      // put points into point cloud
      cloud_voxeled->width = point_vec.size();
      cloud_voxeled->height = 1;
      for (int i = 0; i < point_vec.size(); i++) {
        cloud_voxeled->push_back(point_vec[i]);
      }

      // publish point cloud
      sensor_msgs::PointCloud2 output_msg;
      toROSMsg(*cloud_voxeled, output_msg);
      output_msg.header.frame_id = base_link_frame;
      output_points_pub.publish(output_msg);
    }
    

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber input_points_sub;
    ros::Publisher output_points_pub;

    //parameters
    std::string base_link_frame;
    float resolution;
  };
} // namespace soma_perception

PLUGINLIB_EXPORT_CLASS(soma_perception::OctreeVoxelGridNodelet, nodelet::Nodelet);
