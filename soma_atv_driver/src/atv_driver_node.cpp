#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <maxon_epos_msgs/MotorStates.h>
//
#include <string>
#include <map>
#include <signal.h>
//
#include <qudpsocket.h>
//
#include "atv_driver/definitions.h"
#include "atv_driver/states/StateBase.h"
#include "atv_driver/states/Stop.h"
#include "atv_driver/states/Starting.h"
#include "atv_driver/states/Travelling.h"
#include "atv_driver/states/Braking.h"
//

#define TIMER_T 0.1 //[sec]

class ATVDriver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;

  ros::Publisher pub_motor_commands;
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_motor_states;

  QUdpSocket *clutch_recv;
  QUdpSocket *clutch_send;

  Definitions_t *data;
  std::map<int, StateBase *> states;
  Stop *stop;
  Starting *starting;
  Travelling *travelling;
  Braking *braking;

public:
  ATVDriver() : nh(ros::NodeHandle()),
                pnh(ros::NodeHandle("~"))
  {
    pub_motor_commands = nh.advertise<maxon_epos_msgs::MotorStates>("/set_all_state", 1);
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",
                                                     1,
                                                     &ATVDriver::callback_cmd_vel,
                                                     this);
    sub_motor_states = nh.subscribe<maxon_epos_msgs::MotorStates>("/get_all_state",
                                                                  1,
                                                                  &ATVDriver::callback_motor_states,
                                                                  this);

    data = new Definitions_t();
    data->state = State::Stop; //initial state
    data->cmd_v = 0.0;
    data->current_positions = new double[4]{0.0};
    data->target_positions = new double[4]{0.0};
    data->target_velocity = new long[4]{3500}; //initial values
    data->clutch = data->clutch_cmd = Clutch::Free;

    clutch_recv = new QUdpSocket();
    clutch_recv->bind(12345);
    clutch_send = new QUdpSocket();

    stop = new Stop();
    starting = new Starting();
    travelling = new Travelling();
    braking = new Braking();
    states[State::Stop] = stop;
    states[State::Starting] = starting;
    states[State::Travelling] = travelling;
    states[State::Braking] = braking;

    ROS_INFO("Start timer callback");
    timer = nh.createTimer(ros::Duration((double)TIMER_T),
                           &ATVDriver::main,
                           this);
  }

  ~ATVDriver()
  {
  }

  void shutdown()
  {
    ROS_INFO("ATV Driver shutdown process");

    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position = 0.0; //steering [rad]
    motor_cmd.states[1].position = 0.0; //rear brake [rad]
    motor_cmd.states[2].position = 0.0; //front brake [rad]
    motor_cmd.states[3].position = 0.0; //accel throttle [rad]

    motor_cmd.states[0].position = DEG2RAD(data->target_positions[0]);
    motor_cmd.states[1].position = DEG2RAD(data->target_positions[1]);
    motor_cmd.states[2].position = DEG2RAD(data->target_positions[2]);
    motor_cmd.states[3].position = DEG2RAD(data->target_positions[3]);

    pub_motor_commands.publish(motor_cmd);

    return;
  }

private:
  void main(const ros::TimerEvent &e)
  {
    ROS_INFO(State::Str.at(data->state).c_str());

    //data recieve
    recv_clutch_state(data);

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

    //====================================================================
    // send clutch
    clutch_send->writeDatagram((char *)&data->clutch_cmd,
                               sizeof(int),
                               QHostAddress("192.168.1.79"),
                               22345);
    //====================================================================

    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position = 0.0; //steering [rad]
    motor_cmd.states[1].position = 0.0; //rear brake [rad]
    motor_cmd.states[2].position = 0.0; //front brake [rad]
    motor_cmd.states[3].position = 0.0; //accel throttle [rad]

    motor_cmd.states[0].position = DEG2RAD(data->target_positions[0]);
    motor_cmd.states[1].position = DEG2RAD(data->target_positions[1]);
    motor_cmd.states[2].position = DEG2RAD(data->target_positions[2]);
    motor_cmd.states[3].position = DEG2RAD(data->target_positions[3]);

    pub_motor_commands.publish(motor_cmd);
  }
  //
  //
  void callback_cmd_vel(const geometry_msgs::TwistConstPtr &cmd_vel)
  {
    //convert for steering angle, velocity and move direction (Forward,Backward)
    // ROS_INFO("%.2f,%.2f",cmd_vel->linear.x,cmd_vel->angular.z);

    //convert to steering angle phi
    double phi = 0.0;
    if (abs(cmd_vel->angular.z) <= 0.001 || abs(cmd_vel->linear.x) <= 0.001)
    {
      phi = 0.0;
    }
    else
    {
      double _phi = cmd_vel->angular.z * ((double)WHEEL_BASE / cmd_vel->linear.x); //(rad)
      if (abs(_phi) > DEG2RAD(30.0))
      {
        phi = DEG2RAD(30.0);
      }
      else
      {
        phi = asin(_phi);
      }
    }
    // ROS_INFO("Steer:%.2f", phi);

    //set commands
    data->cmd_v = cmd_vel->linear.x;

    return;
  }
  //
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

    //store position value to local member
    data->current_positions[0] = RAD2DEG(motor_states->states[0].position);
    data->current_positions[1] = RAD2DEG(motor_states->states[1].position);
    data->current_positions[2] = RAD2DEG(motor_states->states[2].position);
    data->current_positions[3] = RAD2DEG(motor_states->states[3].position);
  }

  void recv_clutch_state(Definitions_t *data)
  {
    if (clutch_recv->waitForReadyRead(33))
    {
      int recv; //Integer type 4byte date
      while (clutch_recv->bytesAvailable() > 0)
      {
        clutch_recv->readDatagram((char *)&recv, sizeof(int));
      }
      // ROS_INFO("Clutch state: %d", recv);
      data->clutch = recv;
    }
    else
    {
    }
    return;
  }
};

static bool isShutdown = false;

void SignalHander(int sig)
{
  isShutdown = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atv_driver", ros::init_options::NoSigintHandler);
  ROS_INFO("Start ATV driver node");
  ATVDriver driver;

  signal(SIGINT, SignalHander);

  ros::Rate rate(5);
  while (ros::ok())
  {
    if (isShutdown)
    {
      driver.shutdown();
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}