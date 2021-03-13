#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace soma_perception
{
  class PrefilteringNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;

    PrefilteringNodelet() {}
    virtual ~PrefilteringNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      input_points_sub = nh.subscribe("input_points",
                                      3,
                                      &PrefilteringNodelet::cloud_callback,
                                      this);

      output_points_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 3);

      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      //passthrough filter parameters
      use_pt_filter = pnh.param<bool>("use_pt_filter", true);
      pt_min = pnh.param<double>("pt_min", 0.0);
      pt_max = pnh.param<double>("pt_max", 1.0);
      //distance filter parameter
      use_distance_filter = pnh.param<bool>("use_distance_filter", true);
      distance_near_thresh = pnh.param<double>("distance_near_thresh", 1.0);
      distance_far_thresh = pnh.param<double>("distance_far_thresh", 100.0);

      use_down_sampling = pnh.param<bool>("use_down_sampling", true);
      downsample_leaf_size = pnh.param<double>("down_sample_leaf_size", 0.1);

      sor_k = pnh.param<int>("sor_k", 20);
      sor_stddev = pnh.param<double>("sor_stddev", 1.0);
    }

  private:
    void cloud_callback(pcl::PointCloud<PointT>::ConstPtr input)
    {
      //if point cloud empty
      if (input->empty())
      {
        return;
      }

      pcl::PointCloud<PointT>::Ptr cloud_transformed;
      cloud_transformed.reset(new pcl::PointCloud<PointT>);
      cloud_transformed->clear();
      transform_pointCloud(input, *cloud_transformed); //transform

      pcl::PointCloud<PointT>::Ptr filtered;
      filtered.reset(new pcl::PointCloud<PointT>());
      pcl::copyPointCloud(*cloud_transformed, *filtered);


      //好きなフィルタリング
      // pt_filter(filtered);
      downsample(filtered);

      // pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(input);
      // filtered = downsample(filtered);
      // filtered = sor(filtered);

      output_points_pub.publish(filtered);
    }

    //--------------------------------------------------
    //Transform function
    //--------------------------------------------------
    void transform_pointCloud(pcl::PointCloud<PointT>::ConstPtr input,
                              pcl::PointCloud<PointT> &output)
    {
      if (!base_link_frame.empty())
      {
        if (!tf_listener.canTransform(base_link_frame, input->header.frame_id, ros::Time(0)))
        {
          NODELET_WARN("cannot transform %s->%s", input->header.frame_id.c_str(), base_link_frame.c_str());
          return;
        }

        //get transform
        tf::StampedTransform transform;
        tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);
        //apply transform
        pcl_ros::transformPointCloud(*input, output, transform);
        output.header.frame_id = base_link_frame;
        output.header.stamp = input->header.stamp;
        NODELET_INFO("transformed point cloud (frame_id=%s)", output.header.frame_id.c_str());
      }
    }

    //--------------------------------------------------
    //Passthrough filter function
    //--------------------------------------------------
    
    void pt_filter(const pcl::PointCloud<PointT>::Ptr &src)
    {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(src);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(pt_min, pt_max);
      pass.filter(*filtered);
      pcl::copyPointCloud<PointT>(*filtered, *src);     
    }

    //--------------------------------------------------
    //Distance filter function
    //--------------------------------------------------
    pcl::PointCloud<PointT>::ConstPtr distance_filter(
        const pcl::PointCloud<PointT>::ConstPtr &src) const
    {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      filtered->reserve(src->size());

      if (!use_distance_filter)
      {
        pcl::copyPointCloud(*src, *filtered);
        return filtered;
      }

      std::copy_if(src->begin(), src->end(),
                   std::back_inserter(filtered->points), [&](const PointT &p) {
      double d = p.getVector3fMap().norm();
      return d > distance_near_thresh && d < distance_far_thresh; });

      filtered->width = filtered->size();
      filtered->height = 1;
      filtered->header = src->header;
      return filtered;
    }

    //--------------------------------------------------
    //Down sampling function
    //--------------------------------------------------
    void downsample(const pcl::PointCloud<PointT>::Ptr &src)
    {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

      pcl::VoxelGrid<PointT> filter;
      filter.setInputCloud(src);
      filter.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
      filter.filter(*filtered);
      pcl::copyPointCloud(*filtered, *src);
    }

    //--------------------------------------------------
    //Statistical Outlier Removal function
    //--------------------------------------------------
    pcl::PointCloud<PointT>::ConstPtr sor(const pcl::PointCloud<PointT>::ConstPtr &src) const
    {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

      pcl::StatisticalOutlierRemoval<PointT> filter;
      filter.setInputCloud(src);
      filter.setMeanK(sor_k);
      filter.setStddevMulThresh(sor_stddev);
      filter.filter(*filtered);
      return filtered;
    }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber input_points_sub;
    ros::Publisher output_points_pub;
    tf::TransformListener tf_listener;

    //parameters
    std::string base_link_frame;
    
    bool use_pt_filter;
    double pt_min;
    double pt_max;

    bool use_distance_filter;
    double distance_near_thresh;
    double distance_far_thresh;

    bool use_down_sampling;
    double downsample_leaf_size;

    int sor_k;
    double sor_stddev;
  };
} // namespace soma_perception

PLUGINLIB_EXPORT_CLASS(soma_perception::PrefilteringNodelet, nodelet::Nodelet);
