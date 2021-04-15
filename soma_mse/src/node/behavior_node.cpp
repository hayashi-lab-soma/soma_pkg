#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <math.h>

#include "./soma_mse/definition.h"
#include "./soma_mse/StateBase/StateBase.h"
#include "./soma_mse/States/Stop.h"
#include "./soma_mse/States/MoveTo.h"
#include "./soma_mse/States/Home.h"

#define TIMER_T 0.1 //[sec]

class Behavior
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;

  //frame ids
  std::string base_link_id;
  std::string map_frame_id;
  std::string odom_frame_id;

  //Subscriber
  ros::Subscriber clicked_point_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber command_sub;
  ros::Subscriber odom_sub;

  //Publisher
  ros::Publisher state_pub;
  ros::Publisher pg_pub;
  ros::Publisher xt_pub;
  ros::Publisher ut_pub;

  //local members
  Data_t data;
  //state objects
  std::map<int, StateBase *> states;
  Stop *stop;
  MoveTo *moveto;
  Home *home;

public:
  Behavior() : nh(ros::NodeHandle()),
               pnh(ros::NodeHandle("~"))
  {
    //frame id strings
    base_link_id = pnh.param<std::string>("base_frame_id", "base_link");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    map_frame_id = pnh.param<std::string>("map_frame_id", "map");

    //Subscriber
    clicked_point_sub = nh.subscribe<geo_msgs::PointStamped>("/clicked_point", 1,
                                                             &Behavior::clicked_point_callback, this);
    goal_sub = nh.subscribe<geo_msgs::PoseStamped>("/move_base_simple/goal", 3,
    &Behavior::callback_goal, this);

    command_sub = nh.subscribe<std_msgs::String>("/soma_command", 1,
                                                 &Behavior::command_callback, this);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 3,
                                                &Behavior::odom_callback, this);

    //Publisher
    state_pub = nh.advertise<std_msgs::String>("/soma_state", 3); //state str publisher
    pg_pub = nh.advertise<geo_msgs::PointStamped>("/soma_pg", 3); //global target position publisher
    xt_pub = nh.advertise<geo_msgs::PoseStamped>("/soma_xt", 3);  //state vector publisher
    ut_pub = nh.advertise<geo_msgs::Twist>("/soma_ut", 3);        //control input publisher

    //instances
    data = Data_t();
    data.tfBuf = new tf2_ros::Buffer(ros::Duration(2.0));
    data.tfListener = new tf2_ros::TransformListener(*data.tfBuf);

    ROS_INFO("Wait for tf between base_link and map");
    try
    {
      data.transform_map2base = data.tfBuf->lookupTransform("base_link",
                                                            map_frame_id,
                                                            ros::Time(0),
                                                            ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    data.local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *data.tfBuf);
    data.local_planner = new dwa_local_planner::DWAPlannerROS();
    data.local_planner->initialize("dwa_local_planner", data.tfBuf, data.local_costmap);
    data.global_costmap = new costmap_2d::Costmap2DROS("global_costmap", *data.tfBuf);
    data.global_planner = new navfn::NavfnROS();
    data.global_planner->initialize("global_planner", data.global_costmap);
    data.fixed_start.header.frame_id = base_link_id;
    data.fixed_start.pose.position.x = 0.0;
    data.fixed_start.pose.position.y = 0.0;
    data.fixed_start.pose.position.z = 0.0;
    data.fixed_start.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    data.fixed_target.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    stop = new Stop();
    moveto = new MoveTo(1.0);
    home = new Home(0.5);
    states[State::Stop] = stop;
    states[State::MoveTo] = moveto;
    states[State::GoHome] = home;

    ROS_INFO("Start timer callback");
    timer = nh.createTimer(ros::Duration((double)TIMER_T),
                           &Behavior::main,
                           this);
  }

private:
  void main(const ros::TimerEvent &e)
  {
    //log info
    ROS_INFO("State:%s / Command:%s",
             State::Str.at(data.state).c_str(),
             Command::Str.at(data.command).c_str());

    //
    //
    ROS_INFO("update transforms");
    try
    {
      data.transform_map2base = data.tfBuf->lookupTransform(base_link_id,
                                                            map_frame_id,
                                                            ros::Time(0),
                                                            ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }

    //
    //fms
    int new_state = states[data.state]->Transition(&data);
    if (new_state != data.state)
    {
      states[data.state]->Exit(&data);
      states[new_state]->Enter(&data);
      data.state = new_state;
    }
    else
    {
      states[data.state]->Process(&data);
    }

    //Publish
    std_msgs::String smsgs;
    smsgs.data = State::Str.at(data.state) + "/" + State::Str.at(data.command);
    state_pub.publish(smsgs);

    pg_pub.publish(data.pg);

    ut_pub.publish(data.u_t);
    return;
  }

  void clicked_point_callback(const geometry_msgs::PointStampedConstPtr &msg)
  {
    if (msg->header.frame_id != map_frame_id)
    {
      ROS_WARN("Check global frame of /clicked_point");
      return;
    }

    ROS_INFO("Subscribe target point:%.1f,%.1f,%.1f",
             msg->point.x, msg->point.y, msg->point.z);

    // data.pg = msg->point;
    data.pg = *msg;
    data.pg.header.frame_id = msg->header.frame_id;
    data.pg.header.stamp = ros::Time::now();
    return;
  }
  void callback_goal(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    data.goal = *msg;

    geometry_msgs::PoseStamped source_pose;
    geometry_msgs::PoseStamped start_pose;
    source_pose.header.frame_id = "base_link";
    source_pose.header.stamp = ros::Time(0);
    source_pose.pose.orientation.w = 1.0;

    try{
      start_pose = data.tfBuf->transform(source_pose, "map");
    }
    catch(...){
      ROS_WARN("Error");
    }

    if(!data.global_planner->makePlan(
      start_pose,
        data.goal,
        data.goal_plan)){
      ROS_WARN("Failure global plann");
      exit(1);
    };

        data.local_planner->setPlan(data.goal_plan);
  }

  void command_callback(const std_msgs::StringConstPtr &msg)
  {
    ROS_INFO("Command: %s", msg->data.c_str());

    if (msg->data == "stop")
    {
      data.command = Command::Stop;
    }
    else if (msg->data == "moveto")
    {
      data.command = Command::MoveTo;
    }
    else if (msg->data == "gohome")
    {
      data.command = Command::GoHome;
    }
    return;
  }

  void odom_callback(const nav_msgs::OdometryConstPtr &msg)
  {
    data.x_t = msg->pose.pose;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior");
  ROS_INFO("Start behavior node");
  Behavior behavior;
  ros::spin();
  return 0;
}
