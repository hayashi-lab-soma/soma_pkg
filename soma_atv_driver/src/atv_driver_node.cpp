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

class ATVDriver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;
  double dt;

  //publihser and subscriber
  ros::Publisher pub_motor_states;
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_motor_states;

  //networks
  QUdpSocket *clutch_recv;
  QUdpSocket *clutch_send;

  soma_atv_driver::Data_t *data;

  //state machine and states
  std::map<int, StateBase *> states;
  Stop *stop;
  Starting *starting;
  Travelling *travelling;
  Braking *braking;

public:
  //============================================================
  // constructor
  ATVDriver()
      : nh(ros::NodeHandle()),
        pnh(ros::NodeHandle("~"))
  {
    //============================================================
    // get parameters
    get_parameters(pnh);

    //============================================================
    // publishers
    // publiser for motor states
    pub_motor_states = nh.advertise<maxon_epos_msgs::MotorStates>("/set_all_states", 3);
    //============================================================
    //
    //============================================================
    // subscribers
    // subscriber for cmd_vel:Twist
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",
                                                     3,
                                                     &ATVDriver::callback_cmd_vel,
                                                     this);
    // subscriber for motor states
    sub_motor_states = nh.subscribe<maxon_epos_msgs::MotorStates>("/get_all_state",
                                                                  3,
                                                                  &ATVDriver::callback_motor_states,
                                                                  this);
    //============================================================
    //
    data = new soma_atv_driver::Data_t();
    data->state = State::Stop; //initial state
    data->u_in.v = 0.0;
    data->u_in.phi = 0.0;

    data->current_positions = new double[4]{0.0};
    data->target_positions = new double[4]{0.0};
    data->target_velocity = new long[4]{3500}; //initial values
    data->clutch = data->clutch_cmd = Clutch::Free;

    clutch_recv = new QUdpSocket();
    clutch_recv->bind(Clutch::RecvPort);
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
    timer = nh.createTimer(ros::Duration(dt),
                           &ATVDriver::main,
                           this);
  }

  //============================================================
  // destructor
  ~ATVDriver()
  {
  }

  void shutdown()
  {
    ROS_INFO("ATV Driver shutdown process");

    timer.stop();

    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position = DEG2RAD(0.0);
    motor_cmd.states[1].position = DEG2RAD(0.0);
    motor_cmd.states[2].position = DEG2RAD(0.0);
    motor_cmd.states[3].position = DEG2RAD(0.0);

    pub_motor_states.publish(motor_cmd);

    return;
  }

private:
  void get_parameters(ros::NodeHandle pnh)
  {
    std::vector<std::string> motor_names;
    if (!pnh.getParam("motor_name", motor_names))
    {
      ROS_FATAL("Failed to load motor_names");
      exit(255);
    }

    dt = pnh.param<double>("timer_dt", 0.1);

    return;
  }

  void main(const ros::TimerEvent &e)
  {
    // ROS_INFO(State::Str.at(data->state).c_str());

    //====================================================================
    recv_clutch_state(data);
    //====================================================================

    //finite state machine
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
    send_clutch_state(data);
    //====================================================================

    //====================================================================
    // publish motor target states
    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position = DEG2RAD(data->target_positions[0]); //steering
    motor_cmd.states[1].position = DEG2RAD(data->target_positions[1]); //rear brake
    motor_cmd.states[2].position = DEG2RAD(data->target_positions[2]); //front brake
    motor_cmd.states[3].position = DEG2RAD(data->target_positions[3]); //throttle

    pub_motor_states.publish(motor_cmd);
    //====================================================================
  }
  //
  //
  void callback_cmd_vel(const geometry_msgs::TwistConstPtr &cmd_vel)
  {
    //convert for steering angle, velocity and move direction (Forward,Backward)
    // ROS_INFO("%.2f,%.2f",cmd_vel->linear.x,cmd_vel->angular.z);

    //convert to steering angle phi
    // double phi = 0.0;
    // if (abs(cmd_vel->angular.z) <= 0.001 || abs(cmd_vel->linear.x) <= 0.001)
    // {
    //   phi = 0.0;
    // }
    // else
    // {
    //   double _phi = cmd_vel->angular.z * ((double)WHEEL_BASE / cmd_vel->linear.x); //(rad)
    //   if (abs(_phi) > DEG2RAD(30.0))
    //   {
    //     phi = DEG2RAD(30.0);
    //   }
    //   else
    //   {
    //     phi = asin(_phi);
    //   }
    // }
    // ROS_INFO("Steer:%.2f", phi);

    //set commands
    data->u_in.v = cmd_vel->linear.x;
    data->u_in.phi = angular_vel_to_steering_angle(cmd_vel->linear.x,
                                                   cmd_vel->angular.z); //defined in definitions.h

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
    // ROS_INFO(str_motor_states.c_str());

    //store position value to local member
    data->current_positions[0] = RAD2DEG(motor_states->states[0].position);
    data->current_positions[1] = RAD2DEG(motor_states->states[1].position);
    data->current_positions[2] = RAD2DEG(motor_states->states[2].position);
    data->current_positions[3] = RAD2DEG(motor_states->states[3].position);
  }

  void recv_clutch_state(soma_atv_driver::Data_t *data)
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

  void send_clutch_state(soma_atv_driver::Data_t *data)
  {
    clutch_send->writeDatagram((char *)&data->clutch_cmd,
                               sizeof(int),
                               QHostAddress(QString(Clutch::IP.c_str())),
                               Clutch::SendPort);
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
  while (1)
  {
    if (isShutdown)
    {
      driver.shutdown();
      ros::spinOnce();
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}