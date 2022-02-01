#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>

#include <pcl/exceptions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <math.h>

namespace soma_perception
{
  typedef pcl::PointXYZRGB PointT;

  class DiameterEstimationNodelet : public nodelet::Nodelet
  {
  public:
    DiameterEstimationNodelet(){};
    virtual ~DiameterEstimationNodelet(){};

    virtual void onInit()
    {
      NODELET_INFO("Initializing DiameterEstimationNodelet");

      nh = getNodeHandle();
      nh2 = getNodeHandle();
      pnh = getPrivateNodeHandle();
      initialize_params();
      pub = nh.advertise<sensor_msgs::PointCloud2>("cylinder", 3);
      pub_centers = nh2.advertise<geometry_msgs::PoseArray>("trees_centers2", 1);
      points_sub = nh.subscribe("input_points", 3, &DiameterEstimationNodelet::callback, this);
    }

    void initialize_params()
    {
      base_link_frame = pnh.param<std::string>("base_link", "soma_link");
      normal_distance_weight = pnh.param<double>("normal_distance_weight", 0.1);
      max_iterations = pnh.param<double>("max_iterations", 10000);
      distance_thres = pnh.param<double>("distance_thres", 0.05);
      radius_min = pnh.param<double>("radius_min", 1);
      radius_max = pnh.param<double>("radius_max", 3);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &_input)
    {
      NODELET_INFO("point size: %d", _input->data.size());
      pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<PointT>::Ptr pc_cylinder(new pcl::PointCloud<PointT>());

      pcl::fromROSMsg(*_input, *input);
      transform_pointCloud(input, *cloud_transformed);
      if(cloud_transformed->empty()) return;
      estimate_normal(cloud_transformed, cloud_normals);
      std::vector<float> coeffs = segment_cylinder(cloud_transformed, cloud_normals, pc_cylinder);

      //--------------------------------------------------
      // calcurate diameter
      //--------------------------------------------------
      pcl::PointXYZRGB minPt, maxPt;
      pcl::getMinMax3D(*pc_cylinder, minPt, maxPt);
      double diameter = maxPt.y - minPt.y;

      //--------------------------------------------------
      // publish topic and print logs
      //--------------------------------------------------
      sensor_msgs::PointCloud2 pc_output;
      pcl::toROSMsg(*pc_cylinder, pc_output);
      pc_output.header.frame_id = base_link_frame;
      pub.publish(pc_output);
      NODELET_INFO("pub points size: %5d", (int)pc_cylinder->size());
      NODELET_INFO("radius: %5lf [m]", coeffs[6]);
      NODELET_INFO("center_x: %5lf [m]", coeffs[0]);
      NODELET_INFO("center_y: %5lf [m]", coeffs[1]);
      NODELET_INFO("--------------------------");
      geometry_msgs::PoseArray trees_centers;
      geometry_msgs::Pose pose;
      pose.position.x = coeffs[0];
      pose.position.y = coeffs[1];
      pose.position.z = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      trees_centers.header.frame_id = "velodyne";
      trees_centers.poses.push_back(pose);
      pub_centers.publish(trees_centers);
    }

      void transform_pointCloud(pcl::PointCloud<PointT>::Ptr input,
                                pcl::PointCloud<PointT> &output)
      {
        if (!base_link_frame.empty())
        {
          //get transform
          tf::StampedTransform transform;
          tf_listener.waitForTransform(base_link_frame, input->header.frame_id, ros::Time(0), ros::Duration(10.0));
          tf_listener.lookupTransform(base_link_frame, input->header.frame_id, ros::Time(0), transform);
          //apply transform
          pcl_ros::transformPointCloud(*input, output, transform);
          output.header.frame_id = base_link_frame;
          output.header.stamp = input->header.stamp;
        }
      }

      void estimate_normal(pcl::PointCloud<PointT>::Ptr input,
                          pcl::PointCloud<pcl::Normal>::Ptr output_normal)
      {
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        ne.setSearchMethod(tree);
        ne.setInputCloud(input);
        ne.setKSearch(50);
        ne.compute(*output_normal);
      }

      std::vector<float> segment_cylinder(pcl::PointCloud<PointT>::Ptr input,
                      pcl::PointCloud<pcl::Normal>::Ptr input_normals,
                      pcl::PointCloud<PointT>::Ptr output)
      {
        // if (input->size() < 10)
          // return 0;
          // return NULL;
        //instance of RANSAC segmentation processing object
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sacseg;
        pcl::ExtractIndices<PointT> EI;
        pcl::PointIndices::Ptr inliers;
        pcl::ModelCoefficients::Ptr coeffs;
        inliers.reset(new pcl::PointIndices());
        coeffs.reset(new pcl::ModelCoefficients());
        //set RANSAC parameters
        sacseg.setOptimizeCoefficients (true);
        sacseg.setModelType (pcl::SACMODEL_CYLINDER);
        sacseg.setMethodType (pcl::SAC_RANSAC);
        sacseg.setNormalDistanceWeight (normal_distance_weight);
        sacseg.setMaxIterations (max_iterations);
        sacseg.setDistanceThreshold (distance_thres);
        sacseg.setRadiusLimits (radius_min, radius_max);
        sacseg.setInputCloud (input);
        sacseg.setInputNormals (input_normals);
        try
        {
          sacseg.segment(*inliers, *coeffs);
          EI.setInputCloud(input);
          EI.setIndices(inliers);
          EI.setNegative(false);
          EI.filter(*output);
        }
        catch (const pcl::PCLException &e)
        {
          NODELET_WARN("Cylinder Model Detection Error");
          // return 0; //failure
          // return NULL;
        }
        return coeffs->values; //success
      }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh2;
    ros::NodeHandle pnh;
    tf::TransformListener tf_listener;
    ros::Subscriber points_sub;
    ros::Publisher pub;
    ros::Publisher pub_centers;

    std::string base_link_frame;
    double normal_distance_weight;
    double max_iterations;
    double distance_thres;
    double radius_min;
    double radius_max;
  };
}
PLUGINLIB_EXPORT_CLASS(soma_perception::DiameterEstimationNodelet, nodelet::Nodelet)