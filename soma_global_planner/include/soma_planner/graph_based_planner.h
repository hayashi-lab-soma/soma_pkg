#pragma once
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/GetPlan.h>

namespace graph_based_planner
{
  class GraphBasedPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    GraphBasedPlanner();
    GraphBasedPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    ~GraphBasedPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);

  private:
    costmap_2d::Costmap2DROS* _costmap_ros;
    costmap_2d::Costmap2D* _costmap;
    base_local_planner::WorldModel* world_model;
    bool initialized;

    ros::NodeHandle nh;
  };
};

PLUGINLIB_EXPORT_CLASS(graph_based_planner::GraphBasedPlanner, nav_core::BaseGlobalPlanner)