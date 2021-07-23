#include <soma_planner/graph_based_planner.h>

namespace graph_based_planner
{
  GraphBasedPlanner::GraphBasedPlanner()
      : _costmap_ros(NULL), _costmap(NULL), world_model(NULL), initialized(false)
  {
  }

  GraphBasedPlanner::GraphBasedPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : _costmap_ros(NULL), _costmap(NULL), world_model(NULL), initialized(false)
  {
    initialize(name, costmap_ros);
  }

  GraphBasedPlanner::~GraphBasedPlanner()
  {
    delete this->world_model;
  }

  void GraphBasedPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized)
    {
      ROS_INFO("Initialize the global planner");
      _costmap_ros = costmap_ros;
      _costmap = _costmap_ros->getCostmap();
      nh = ros::NodeHandle();
      ros::NodeHandle pnh("~" + name);
      world_model = new base_local_planner::CostmapModel(*_costmap);

      initialized = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized ...");
    }
  }

  bool GraphBasedPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                   const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!initialized)
    {
      ROS_ERROR("The planner has not been initialized");
      return false;
    }

    plan.clear();
    _costmap = _costmap_ros->getCostmap();

    // call service of python script?
    // check start pose
    ROS_INFO("%.f, %.f, %.f",
             start.pose.position.x,
             start.pose.position.y,
             start.pose.position.z);

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("get_graph_based_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = 1.0;
    if (client.call(srv))
    {
      ROS_INFO("%d", (int)srv.response.plan.poses.size());
      plan = srv.response.plan.poses;
    }
    else{
    }
    return true;
  }
};
