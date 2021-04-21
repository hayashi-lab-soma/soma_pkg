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
      ROS_INFO("Initialize the planner");
      _costmap_ros = costmap_ros;
      _costmap = _costmap_ros->getCostmap();

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

    return true;
  }
};
