#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/exceptions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/shared_ptr.hpp>
#include <time.h>


namespace soma_perception
{
  typedef pcl::PointXYZRGB PointT;

  class SampleSegmentationNodelet : public nodelet::Nodelet
  {
  public:
    SampleSegmentationNodelet(){
    }
    virtual ~SampleSegmentationNodelet(){      
    }

    virtual void onInit()
    {
      NODELET_INFO("InitializingSampleSegmentationNodelet");

      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      points_sub = nh.subscribe("input_points",
      3,
      &SampleSegmentationNodelet::callback,
      this);

      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      number_of_neighbours = pnh.param<double>("number_of_neighbours", 30);
      smoothThreshold = pnh.param<double>("smoothThreshold", 3.0);
      curvatureThreshold = pnh.param<double>("curvatureThreshold", 1.0);
      output_dirpath = pnh.param<std::string>("output_dirpath", "/home/soma/Documents/tomokawa/pcd/20201212_rgs/");
      export_switch = pnh.param<bool>("export_switch", false);
      segmented_pub = nh.advertise<sensor_msgs::PointCloud2>("segmented_points", 3);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &_input){
      NODELET_INFO("Point size:%d", _input->data.size());

      pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*_input, *input);

      //--------------------------------------------------
      // transform pointcloud
      //--------------------------------------------------
      pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
      transform_pointCloud(input, *cloud_transformed);
      if(cloud_transformed->empty()) return;

      //--------------------------------------------------
      // RegionGrowing
      //--------------------------------------------------
      pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
      pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
      pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
      normal_estimator.setSearchMethod (tree);
      normal_estimator.setInputCloud (cloud_transformed);
      normal_estimator.setKSearch (50);
      normal_estimator.compute (*normals);

      pcl::RegionGrowing<PointT, pcl::Normal> reg;
      reg.setMinClusterSize (50);
      reg.setMaxClusterSize (1000000);
      reg.setSearchMethod (tree);
      reg.setNumberOfNeighbours (number_of_neighbours);
      reg.setInputCloud (cloud_transformed);
      //reg.setIndices (indices);
      reg.setInputNormals (normals);
      reg.setSmoothnessThreshold (smoothThreshold / 180.0 * M_PI);
      // reg.setSmoothnessThreshold (smoothThreshold);
      reg.setCurvatureThreshold (curvatureThreshold);

      std::vector <pcl::PointIndices> clusters;
      reg.extract (clusters);

      std::cout << "Number of clusters is" << clusters.size () << std::endl;
      std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;

      pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloud();
      sensor_msgs::PointCloud2 pub;
      pcl::toROSMsg(*colored_cloud, pub);
      pub.header.frame_id = base_link_frame;
      pub.header.stamp = ros::Time::now();
      segmented_pub.publish(pub);

      //--------------------------------------------------
      // export pcd_file
      //--------------------------------------------------
      if(export_switch) {
        for (int i=0; i < clusters.size(); i++) {
          pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>);
          pcl::copyPointCloud(*cloud_transformed, clusters[i].indices, *tmp);
          std::string ros_now = std::to_string(ros::Time::now().toSec());
          std::string id = std::to_string(i);
          std::string save_file_name = output_dirpath + ros_now + "_" + id + ".pcd";
          
          try{
            pcl::io::savePCDFileBinary(save_file_name, *tmp);
          }
          catch(pcl::PCLException::exception &e){
            NODELET_WARN(e.what());
          }
        }
      }


      return;
    }

    void transform_pointCloud(pcl::PointCloud<PointT>::Ptr input,
                              pcl::PointCloud<PointT> &output)
    {
      if (!base_link_frame.empty())
      {
        // if (!tf_listener.canTransform(base_link_frame, input->header.frame_id, ros::Time(0)))
        // {
        //   NODELET_WARN("cannot transform %s->%s", input->header.frame_id.c_str(), base_link_frame.c_str());
        //   return;
        // }

        //get transform
        tf::StampedTransform transform;
        tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);
        //apply transform
        pcl_ros::transformPointCloud(*input, output, transform);
        output.header.frame_id = base_link_frame;
        output.header.stamp = input->header.stamp;
        // NODELET_INFO("transformed point cloud (frame_id=%s)", output.header.frame_id.c_str());
      }
    }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    tf::TransformListener tf_listener;

    ros::Subscriber points_sub;

    ros::Publisher segmented_pub;

    std::string base_link_frame;
    double number_of_neighbours;
    double smoothThreshold;
    double curvatureThreshold;
    std::string output_dirpath;
    bool export_switch;
  };
}

PLUGINLIB_EXPORT_CLASS(soma_perception::SampleSegmentationNodelet, nodelet::Nodelet)
