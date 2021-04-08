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
//

class ATVDriver {
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;
  double dt;

  // publihser and subscriber
  ros::Publisher pub_action;
  ros::Publisher pub_action_str; // State::Start,...
  ros::Publisher pub_motor_states;
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_wheel_vel; // test for
  ros::Subscriber sub_motor_states;

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
  //============================================================
  // constructor
  ATVDriver() : nh(ros::NodeHandle()), pnh(ros::NodeHandle("~")) {
    //============================================================
    // get parameters
    get_parameters(pnh);

    //============================================================
    // publishers
    pub_action = nh.advertise<std_msgs::Int32>("/soma/action", 3);
    pub_action_str = nh.advertise<std_msgs::String>("/soma/action_str", 3);
    // publiser for motor states
    pub_motor_states =
        nh.advertise<maxon_epos_msgs::MotorStates>("/set_all_states", 3);
    //============================================================
    //
    //============================================================
    // subscribers
    // subscriber for cmd_vel:Twist
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 3, &ATVDriver::callback_cmd_vel, this);
    //
    sub_wheel_vel = nh.subscribe<std_msgs::Float32>(
        "/soma/wheel_vel", 3, &ATVDriver::callback_wheel_vel, this);
    // subscriber for motor states
    sub_motor_states = nh.subscribe<maxon_epos_msgs::MotorStates>(
        "/get_all_state", 3, &ATVDriver::callback_motor_states, this);
    //============================================================
    //

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
    timer = nh.createTimer(ros::Duration(dt), &ATVDriver::main, this);
  }

  //============================================================
  // destructor
  ~ATVDriver() {}

  void shutdown() {
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
  void get_parameters(ros::NodeHandle pnh) {
    // std::vector<std::string> motor_names;
    // if (!pnh.getParam("motor_name", motor_names))
    // {
    //   ROS_FATAL("Failed to load motor_names");
    //   exit(255);
    // }

    dt = pnh.param<double>("timer_dt", 0.1);

    data = new soma_atv_driver::Data_t();
    data->dt = dt;
    data->state = State::Stop; // initial state
    data->u_in.v = 0.0;
    data->u_in.phi = 0.0;

    data->current_positions = new double[4]{0.0};
    data->target_positions = new double[4]{0.0};
    data->target_velocity =
        new long[4]{3500, 3500, 3500, 3500}; // initial values
    data->clutch = data->clutch_cmd = Clutch::Free;

    data->wheel_vel = 0.0;
    data->ev = new double[3]{0.0};
    data->P = pnh.param<double>("P", 0.1);
    data->D = pnh.param<double>("D", 0.1);

    data->rear_brake_slow_open_rpm =
        pnh.param<double>("rear_brake_slow_open_rpm", 100.0);
    data->throttle_offset = pnh.param<double>("throttle_offset", 7.0);
    data->throttle_max = pnh.param<double>("throttle_max", 13.0);

    return;
  }

  void main(const ros::TimerEvent &e) {
    // ROS_INFO(State::Str.at(data->state).c_str());

    data->ev[2] = data->ev[1];
    data->ev[1] = data->ev[0];
    data->ev[0] = data->wheel_vel - 1.0;

    //====================================================================
    recv_clutch_state(data);
    //====================================================================

    // finite state machine
    int new_state = states[data->state]->Transition(data);
    if (new_state != data->state) {
      states[data->state]->Exit(data);
      states[new_state]->Enter(data);
      data->state = new_state;
    } else {
      states[data->state]->Process(data);
    }

    //====================================================================
    send_clutch_state(data);
    //====================================================================

    //====================================================================
    // publish current states for logging
    std_msgs::Int32 action;
    action.data = data->state;
    pub_action.publish(action);

    std_msgs::String action_str;
    std::stringstream ss;
    ss << State::Str.at(data->state);
    ss << "," << data->P;
    ss << "," << data->D;
    action_str.data = std::string(ss.str());
    pub_action_str.publish(action_str);

    // publish motor target states
    maxon_epos_msgs::MotorStates motor_cmd;
    motor_cmd.states.resize(4);
    motor_cmd.header.stamp = ros::Time::now();

    motor_cmd.states[0].position =
        DEG2RAD(data->target_positions[0]); // steering
    motor_cmd.states[1].position =
        DEG2RAD(data->target_positions[1]); // rear brake
    motor_cmd.states[2].position =
        DEG2RAD(data->target_positions[2]); // front brake
    motor_cmd.states[3].position =
        DEG2RAD(data->target_positions[3]); // throttle

    motor_cmd.states[0].velocity = data->target_velocity[0];
    motor_cmd.states[1].velocity = data->target_velocity[1];
    motor_cmd.states[2].velocity = data->target_velocity[2];
    motor_cmd.states[3].velocity = data->target_velocity[3];

    pub_motor_states.publish(motor_cmd);
    //====================================================================
  }
  //
  //
  void callback_cmd_vel(const geometry_msgs::TwistConstPtr &cmd_vel) {
    // set commands
    data->u_in.v = cmd_vel->linear.x;
    data->u_in.phi = angular_vel_to_steering_angle(
        cmd_vel->linear.x,
        cmd_vel->angular.z); // defined in definitions.h
    // steering angle limit (-25~25degree)
    data->u_in.phi = std::max(data->u_in.phi, DEG2RAD(-25));
    data->u_in.phi = std::min(data->u_in.phi, DEG2RAD(25));

    return;
  }
  //
  void callback_wheel_vel(const std_msgs::Float32ConstPtr &wheel_vel) {
    ROS_DEBUG("call back wheel velocity: %.2f", wheel_vel->data);
    data->wheel_vel = wheel_vel->data;
  }
  //
  void callback_motor_states(
      const maxon_epos_msgs::MotorStatesConstPtr &motor_states) {
    std::string str_motor_states = "";
    for (int i = 0; i < (int)motor_states->states.size(); i++) {

      str_motor_states += motor_states->states[i].motor_name;
      str_motor_states += ": ";
      str_motor_states += std::to_string(motor_states->states[i].position);
      str_motor_states += "\n";
    }
    // ROS_INFO(str_motor_states.c_str());

    // store position value to local member
    data->current_positions[0] = RAD2DEG(motor_states->states[0].position);
    data->current_positions[1] = RAD2DEG(motor_states->states[1].position);
    data->current_positions[2] = RAD2DEG(motor_states->states[2].position);
    data->current_positions[3] = RAD2DEG(motor_states->states[3].position);
  }

  void recv_clutch_state(soma_atv_driver::Data_t *data) {
    if (clutch_recv->waitForReadyRead(33)) {
      int recv; // Integer type 4byte date
      while (clutch_recv->bytesAvailable() > 0) {
        clutch_recv->readDatagram((char *)&recv, sizeof(int));
      }
      // ROS_INFO("Clutch state: %d", recv);
      data->clutch = recv;
    } else {
    }
    return;
  }

  void send_clutch_state(soma_atv_driver::Data_t *data) {
    clutch_send->writeDatagram((char *)&data->clutch_cmd, sizeof(int),
                               QHostAddress(QString(Clutch::IP.c_str())),
                               Clutch::SendPort);
  }
};

static bool isShutdown = false;

void SignalHander(int sig) { isShutdown = true; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "atv_driver", ros::init_options::NoSigintHandler);
  ROS_INFO("Start ATV driver node");
  ATVDriver driver;

  signal(SIGINT, SignalHander);

  ros::Rate rate(5);
  while (1) {
    if (isShutdown) {
      driver.shutdown();
      ros::spinOnce();
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}