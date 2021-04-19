#include <geometry_msgs/Twist.h>
#include <maxon_epos_msgs/MotorStates.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//
#include <map>
#include <math.h>
#include <signal.h>
#include <string>
//
#include <qudpsocket.h>
//
#include "atv_driver/definitions.h"
#include "atv_driver/states/Braking.h"
#include "atv_driver/states/Starting.h"
#include "atv_driver/states/StateBase.h"
#include "atv_driver/states/Stop.h"
#include "atv_driver/states/Travelling.h"
#include <soma_msgs/SOMAStatus.h>
//

/**
 * @brief
 *
 */
class ATVDriver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;
  double dt;

  // subscribers
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_wheel_vel;
  ros::Subscriber sub_motor_states;

  // publihsers
  ros::Publisher pub_soma_status;
  ros::Publisher pub_motor_states;

  // networks
  QUdpSocket *clutch_recv;
  QUdpSocket *clutch_send;

  soma_atv_driver::Data_t *data;

  // state machine and states
  std::map<int, StateBase *> states;
  Stop *stop;
  Starting *starting;
  Travelling *travelling;
  Braking *braking;

public:
  /**
   * @brief Construct a new ATVDriver object
   *
   */
  ATVDriver() : nh(ros::NodeHandle()), pnh(ros::NodeHandle("~"))
  {
    //============================================================
    // get parameters
    data = new soma_atv_driver::Data_t();
    dt = pnh.param<double>("loop_rate", 0.1);
    data->dt = pnh.param<double>("loop_rate", 0.1);

    data->target_vel = pnh.param<double>("target_vel", 0.8);
    data->motors.steering.Min = pnh.param<double>("steering_pos_min", -25.0);
    data->motors.steering.Max = pnh.param<double>("steering_pos_max", 25.0);
    data->motors.rear_brake.Min = pnh.param<double>("rear_brake_pos_min", 0.0);
    data->motors.rear_brake.Max = pnh.param<double>("rear_brake_pos_max", 10.0);
    data->motors.front_brake.Min =
        pnh.param<double>("front_brake_pos_min", 0.0);
    data->motors.front_brake.Max =
        pnh.param<double>("front_brake_pos_max", 10.0);
    data->motors.throttle.Min = pnh.param<double>("throttle_pos_min", 0.0);
    data->motors.throttle.Max = pnh.param<double>("throttle_pos_max", 12.0);
    data->motors.rear_brake_starting_state_low_rpm =
        pnh.param<double>("rear_brake_starting_state_low_rpm", 100.0);
    data->motors.throttle_regular = pnh.param<double>("throttle_regular", 5.0);
    data->Kp = pnh.param<double>("P", 0.1);
    data->Kd = pnh.param<double>("D", 0.1);

    data->state = State::Init; // initial state
    data->u_in.v = 0.0;
    data->u_in.phi = 0.0;
    data->wheel_vel = 0.0;
    data->ev = new double[3]{0.0};

    //============================================================
    // subscribers
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 3, &ATVDriver::callback_cmd_vel, this);
    sub_wheel_vel = nh.subscribe<std_msgs::Float32>(
        "/wheel_vel", 3, &ATVDriver::callback_wheel_vel, this);
    sub_motor_states = nh.subscribe<maxon_epos_msgs::MotorStates>(
        "/motor_states", 3, &ATVDriver::callback_motor_states, this);
    //============================================================

    //============================================================
    // publishers
    pub_soma_status = nh.advertise<soma_msgs::SOMAStatus>("/soma/status", 3);
    pub_motor_states = nh.advertise<maxon_epos_msgs::MotorStates>(
        "/soma/atv_driver/set_motor_states", 3);
    //============================================================

    //============================================================
    // clutch control client
    clutch_recv = new QUdpSocket();
    clutch_recv->bind(Clutch::RecvPort);
    clutch_send = new QUdpSocket();

    //============================================================
    // make state machine
    stop = new Stop();
    starting = new Starting();
    travelling = new Travelling();
    braking = new Braking();
    states[State::Stop] = stop;
    states[State::Starting] = starting;
    states[State::Travelling] = travelling;
    states[State::Braking] = braking;

    ROS_INFO("Start timer callback");
    timer = nh.createTimer(ros::Duration(dt), &ATVDriver::main, this);
  }

  /**
   * @brief Destroy the ATVDriver object
   *
   */
  ~ATVDriver() {}

  /**
   * @brief
   *
   */
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

    motor_cmd.states[0].velocity = 3500;
    motor_cmd.states[1].velocity = 3500;
    motor_cmd.states[2].velocity = 3500;
    motor_cmd.states[3].velocity = 3500;

    pub_motor_states.publish(motor_cmd);

    return;
  }

private:
  /**
   * @brief Get the parameters object
   *
   * @param pnh
   */
  // void get_parameters(ros::NodeHandle pnh)
  // {
  //   return;
  // }

  /**
   * @brief
   *
   * @param e
   */
  void main(const ros::TimerEvent &e)
  {
    //====================================================================
    // calculate velocity errors
    data->ev[2] = data->ev[1]; // error(t-2)
    data->ev[1] = data->ev[0]; // error(t-1)
    if (abs(data->u_in.v) >= 0.001)
    {
      data->ev[0] = data->target_vel - abs(data->wheel_vel); // error(t)
    }
    else
    {
      data->ev[0] = 0.0 - abs(data->wheel_vel); // error(t)
    }

    //====================================================================

    //====================================================================
    // clutch state recieve
    recv_clutch_state(data);
    //====================================================================

    if (data->state == State::Init)
    {
      states[State::Stop]->Enter(data);
      data->state = State::Stop;
    }
    else
    {
      // finite state machine
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

    //====================================================================
    // clutch command send
    send_clutch_state(data);
    //====================================================================

    //====================================================================
    // publish current states
    soma_msgs::SOMAStatus soma_status;
    soma_status.header.stamp = ros::Time::now();

    soma_status.status = data->state;
    soma_status.status_str = State::Str.at(data->state);

    soma_status.target_vel = data->target_vel;
    soma_status.wheel_vel = data->wheel_vel;
    soma_status.vel_errors.resize(3);
    soma_status.vel_errors[0] = data->ev[0];
    soma_status.vel_errors[1] = data->ev[1];
    soma_status.vel_errors[2] = data->ev[2];
    soma_status.PGain = data->Kp;
    soma_status.DGain = data->Kd;

    soma_status.steering_target_pos = data->motors.steer_pos.In;    // degree
    soma_status.rear_target_pos = data->motors.rear_pos.In;         // degree
    soma_status.front_target_pos = data->motors.front_pos.In;       // degree
    soma_status.throttle_target_pos = data->motors.throttle_pos.In; // degree

    soma_status.steering_pos = data->motors.steer_pos.Out;    // degree
    soma_status.rear_pos = data->motors.rear_pos.Out;         // degree
    soma_status.front_pos = data->motors.front_pos.Out;       // degree
    soma_status.throttle_pos = data->motors.throttle_pos.Out; // degree

    soma_status.clutch_status = data->clutch.out;
    soma_status.clutch_status_str = Clutch::Str.at(data->clutch.out);

    pub_soma_status.publish(soma_status);

    // publish motor target states
    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position =
        DEG2RAD(data->motors.steer_pos.In); // steering
    motor_cmd.states[1].position =
        DEG2RAD(data->motors.rear_pos.In); // rear brake
    motor_cmd.states[2].position =
        DEG2RAD(data->motors.front_pos.In); // front brake
    motor_cmd.states[3].position =
        DEG2RAD(data->motors.throttle_pos.In); // throttle

    motor_cmd.states[0].velocity = data->motors.steer_vel.In;
    motor_cmd.states[1].velocity = data->motors.rear_vel.In;
    motor_cmd.states[2].velocity = data->motors.front_vel.In;
    motor_cmd.states[3].velocity = data->motors.throttle_vel.In;

    pub_motor_states.publish(motor_cmd);
    //====================================================================
  }

  /**
   * @brief
   * set control input for car-like robot
   *
   * @param cmd_vel
   * Input Twist message
   */
  void callback_cmd_vel(const geometry_msgs::TwistConstPtr &cmd_vel)
  {
    // set commands
    data->u_in.v = cmd_vel->linear.x; //
    // data->u_in.v = data->target_vel;  //constant velocity value
    data->u_in.phi = angular_vel_to_steering_angle(
        cmd_vel->linear.x,
        cmd_vel->angular.z); // defined in definitions.h
    // min max limit
    data->u_in.phi =
        std::max(data->u_in.phi, DEG2RAD(data->motors.steering.Min));
    data->u_in.phi =
        std::min(data->u_in.phi, DEG2RAD(data->motors.steering.Max));
    return;
  }

  /**
   * @brief
   * call back function for wheel rotate speed (m/s)
   *
   * @param d
   * Float32 message containing front wheel rotation speed (m/s)
   */
  void callback_wheel_vel(const std_msgs::Float32ConstPtr &d)
  {
    ROS_INFO("wheel rotate velocity: %.2f", d->data);
    data->wheel_vel = d->data; // store
  }

  /**
   * @brief
   *
   * @param motor_states
   */
  void callback_motor_states(
      const maxon_epos_msgs::MotorStatesConstPtr &motor_states)
  {
    // store position value to local member
    data->motors.steer_pos.Out = RAD2DEG(motor_states->states[0].position);
    data->motors.rear_pos.Out = RAD2DEG(motor_states->states[1].position);
    data->motors.front_pos.Out = RAD2DEG(motor_states->states[2].position);
    data->motors.throttle_pos.Out = RAD2DEG(motor_states->states[3].position);
  }
  //-------------------------
  // recv_clutch_state
  //-------------------------
  void recv_clutch_state(soma_atv_driver::Data_t *data)
  {
    if (clutch_recv->waitForReadyRead(33))
    {
      int recv; // Integer type 4byte date
      while (clutch_recv->bytesAvailable() > 0)
      {
        clutch_recv->readDatagram((char *)&recv, sizeof(int));
      }
      data->clutch.out = recv;
    }
    else
    {
      ROS_WARN("%s", clutch_recv->errorString().toStdString().c_str());
    }
    return;
  }

  //-------------------------
  // send_clutch_state
  //
  //-------------------------
  void send_clutch_state(soma_atv_driver::Data_t *data)
  {
    clutch_send->writeDatagram((char *)&data->clutch.in, sizeof(int),
                               QHostAddress(QString(Clutch::IP.c_str())),
                               Clutch::SendPort);
  }
};

static bool isShutdown = false;

void SignalHander(int sig) { isShutdown = true; }

//-------------------------
// main
//-------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "atv_driver", ros::init_options::NoSigintHandler);
  ROS_INFO("Start ATV driver node");

  signal(SIGINT, SignalHander);

  // ros::AsyncSpinner spinner(0);
  // spinner.start();
  ATVDriver driver;
  // ros::waitForShutdown();

  ros::Rate rate(5);
  ros::Duration shutdown_wait(5.0); //(sec)

  while (1)
  {
    ROS_DEBUG("%f", ros::Time::now().toSec());

    if (isShutdown)
    {
      driver.shutdown();
      ros::spinOnce();
      shutdown_wait.sleep();
      break;
    }
    ros::spinOnce();
    rate.sleep();
    // loop_duration.sleep();
  }

  return 0;
}