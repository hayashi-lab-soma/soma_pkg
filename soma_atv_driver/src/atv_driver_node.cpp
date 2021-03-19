#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <maxon_epos_msgs/MotorStates.h>
//
#include <string>
#include <map>
//
#include "atv_driver/definitions.h"
#include "atv_driver/states/StateBase.h"
#include "atv_driver/states/Stop.h"
//

#define TIMER_T 0.1 //[sec]

class ATVDriver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;

  ros::Subscriber sub_motor_states;

  Definitions_t *data;
  std::map<int, StateBase *> states;
  Stop *stop;

public:
  ATVDriver() : nh(ros::NodeHandle()),
                pnh(ros::NodeHandle("~"))
  {
    sub_motor_states = nh.subscribe<maxon_epos_msgs::MotorStates>("/get_all_state",
                                                                  10,
                                                                  &ATVDriver::callback_motor_states,
                                                                  this);

    data = new Definitions_t();
    data->state = State::Stop; //initial state

    stop = new Stop();
    states[State::Stop] = stop;

    ROS_INFO("Start timer callback");
    timer = nh.createTimer(ros::Duration((double)TIMER_T),
                           &ATVDriver::main,
                           this);
  }

  ~ATVDriver()
  {
  }

private:
  void main(const ros::TimerEvent &e)
  {
    ROS_INFO(State::Str.at(data->state).c_str());

    //fms
    int new_state = states[data->state]->Transition(data);
    if (new_state != data->state)
    {
      states[data->state]->Exit(data);
      states[new_state]->Enter(data);
      data->state = new_state;
    }
    else
    {
      states[data->state]->Process(data);
    }
  }

  void callback_motor_states(const maxon_epos_msgs::MotorStatesConstPtr &motor_states)
  {
    std::string str_motor_states = "";
    for (int i = 0; i < (int)motor_states->states.size(); i++)
    {
      str_motor_states += motor_states->states[i].motor_name;
      str_motor_states += ": ";
      str_motor_states += std::to_string(motor_states->states[i].position);
      str_motor_states += "\n";
    }
    ROS_INFO(str_motor_states.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atv_driver");
  ROS_INFO("Start ATV driver node");
  ATVDriver driver;
  ros::spin();
  return 0;
}