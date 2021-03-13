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
#include <pcl/exceptions.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <boost/shared_ptr.hpp>
#include <time.h>

namespace soma_perception
{
  typedef pcl::PointXYZRGB PointT;

  class MlpPrefilteringNodelet : public nodelet::Nodelet
  {
  public:
    MlpPrefilteringNodelet(){};
    virtual ~MlpPrefilteringNodelet(){};

    virtual void onInit()
    {
      NODELET_INFO("Initializing MlpPrefilteringNodelet");

      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_normal", 3);
      points_sub = nh.subscribe("input_points",
      3, 
      &MlpPrefilteringNodelet::callback, 
      this);

      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      search_radius = pnh.param<double>("search_radius", 0.05);

    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &_input)
    {
      // NODELET_INFO("point size: %d", _input->data.size());

      pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*_input, *input);

      //--------------------------------------------------
      // transform pointcloud
      //--------------------------------------------------
      pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
      transform_pointCloud(input, *cloud_transformed);
      if(cloud_transformed->empty()) return;

      //--------------------------------------------------
      // normal_estimation
      //--------------------------------------------------
      pcl::PointCloud<pcl::PointNormal>::Ptr normal_output(new pcl::PointCloud<pcl::PointNormal>);
      estimation_normal(cloud_transformed, normal_output);

      //--------------------------------------------------
      // publish
      //--------------------------------------------------
      sensor_msgs::PointCloud2 pc_normal;
      pcl::toROSMsg(*normal_output, pc_normal);
      pc_normal.header.frame_id = base_link_frame;
      NODELET_INFO("pub points size:%5d", (int)normal_output->size());
      if((int)normal_output->size() >= 4096){
        pub.publish(pc_normal);
      }
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

    void estimation_normal(pcl::PointCloud<PointT>::Ptr input, 
                           pcl::PointCloud<pcl::PointNormal>::Ptr output)
    {
      pcl::NormalEstimationOMP<PointT, pcl::Normal> impl;
      impl.setInputCloud(input);
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      impl.setSearchMethod(tree);
      impl.setRadiusSearch(search_radius);
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
      impl.compute(*normal_cloud);

      output->points.resize(input->points.size());
      for (size_t i = 0; i < output->points.size(); i++) {
        pcl::PointNormal p;
        p.x = input->points[i].x;
        p.y = input->points[i].y;
        p.z = input->points[i].z;
        // p.rgb = input->points[i].rgb;
        p.normal_x = normal_cloud->points[i].normal_x;
        p.normal_y = normal_cloud->points[i].normal_y;
        p.normal_z = normal_cloud->points[i].normal_z;
        output->points[i] = p;
      }
    }
  
  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    tf::TransformListener tf_listener;

    ros::Subscriber points_sub;
    ros::Publisher pub;

    std::string base_link_frame;
    double search_radius;

  };
}
PLUGINLIB_EXPORT_CLASS(soma_perception::MlpPrefilteringNodelet, nodelet::Nodelet)