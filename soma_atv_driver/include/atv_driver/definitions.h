#pragma once

#include <map>
#include <math.h>
#include <string>

#define DEG2RAD(x) (x / 180.0 * M_PI)
#define RAD2DEG(x) (x / M_PI * 180.0)

#define WHEEL_BASE 1.04 //(m)

static double angular_vel_to_steering_angle(double v, double omega) {
  if (abs(omega) <= 0.000001)
    return 0; // zero radian

  double radius = abs(v) / abs(omega);

  if (omega < 0.0) {
    return -atan((double)WHEEL_BASE / radius); // radian
  } else {
    return atan((double)WHEEL_BASE / radius); // radian
  }
}

namespace State {
const int Init = -1;
const int Stop = 0;
const int Starting = 1;
const int Travelling = 2;
const int Braking = 3;

const std::map<int, std::string> Str = {{Stop, "Stop"},
                                        {Starting, "Starting"},
                                        {Travelling, "Travelling"},
                                        {Braking, "Braking"}};
} // namespace State

namespace Clutch {
const std::string IP = "192.168.1.79";
const int RecvPort = 12345;
const int SendPort = 22345;

const int Forward = 1;
const int Reverse = 2;
const int Free = 3;

const std::map<int, std::string> Str{
    {Forward, "Forward"}, {Reverse, "Reverse"}, {Free, "Free"}};
} // namespace Clutch

namespace Rotary {
const std::string IP = "192.168.1.79";
const int RecvPort = 12346;
const int SendPort = 22346;
} // namespace Rotary

/**
 * @brief
 *
 */
namespace soma_atv_driver {
/**
 * @brief
 * control input structure
 */
struct U_t {
  double v;   // linear velocity (m/s)
  double phi; // steering angle (rad)
};

struct Motors_t {
  struct Pos_t {
    double In;
    double Out;
    Pos_t() : In(0.0), Out(0.0) {}
  } steer_pos, rear_pos, front_pos, throttle_pos; // degrees

  struct Vel_t {
    double In;
    double Out;
    Vel_t() : In(0), Out(0) {}
  } steer_vel, rear_vel, front_vel, throttle_vel; // rpm

  struct PositionLimit {
    double Min, Max;
    PositionLimit() : Min(0.0), Max(0.0) {}
  } steering, rear_brake, front_brake, throttle;

  // starting state sparameters
  double rear_brake_starting_state_low_rpm;
  double throttle_regular;
};

struct Clutch_t {
  int in, out;
  Clutch_t() : in(Clutch::Free), out(Clutch::Free) {}
};

/**
 * @brief
 * shared memory data structure
 */
struct Data_t {
  // parameters
  double dt;         // loop duration time (sec)
  double target_vel; // target velocity (m/s)

  Motors_t motors; // motor control parameter
  Clutch_t clutch;

  int state; // state variable (State namespace)
  U_t u_in;  // controll input

  double wheel_vel; // current velocity of wheel (m/s)
  double *ev;       // error of velocity
  double Kp, Kd;    // control gain
};
} // namespace soma_atv_driver
