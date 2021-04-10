#pragma once

#include <string>
#include <map>
#include <math.h>

#define DEG2RAD(x) (x / 180.0 * M_PI)
#define RAD2DEG(x) (x / M_PI * 180.0)

#define WHEEL_BASE 1.04 //(m)

namespace State
{
  const int Stop = 0;
  const int Starting = 1;
  const int Travelling = 2;
  const int Braking = 3;

  const std::map<int, std::string> Str = {
      {Stop, "Stop"},
      {Starting, "Starting"},
      {Travelling, "Travelling"},
      {Braking, "Braking"}};
} // namespace State

// namespace Motor
// {
//   namespace Steering
//   {
//     const double Min = -25.0;
//     const double Max = 25.0;
//   }
//   namespace RearBrake
//   {
//     const double Min = 0.0;
//     const double Max = 10.0;
//   }
//   namespace FrontBrake
//   {
//     const double Min = 0.0;
//     const double Max = 15.0;
//   }
//   namespace Throttle
//   {
//     const double Min = 0.0;
//     const double Max = 20.0;
//   }

// }

namespace Clutch
{
  const std::string IP = "192.168.1.79";
  const int RecvPort = 12345;
  const int SendPort = 22345;

  const int Forward = 1;
  const int Reverse = 2;
  const int Free = 3;

  const std::map<int, std::string> Str{{Forward, "Forward"},
                                       {Reverse, "Reverse"},
                                       {Free, "Free"}};
}

namespace Rotary
{
  const std::string IP = "192.168.1.79";
  const int RecvPort = 12346;
  const int SendPort = 22346;
}

/**
 * @brief 
 * 
 */
namespace soma_atv_driver
{
  /**
   * @brief 
   * control input structure
   */
  struct U_t
  {
    double v;   //linear velocity (m/s)
    double phi; //steering angle (rad)
  };

  struct Motors
  {
    struct PositionLimit
    {
      double Min, Max;
      PositionLimit() : Min(0.0), Max(0.0) {}
    };
    //
    PositionLimit steering, rear_brake, front_brake, throttle;
    //starting parameters
    double rear_brake_starting_state_low_rpm;
    double throttle_regular;
  };

  /**
   * @brief 
   * shared memory data structure
   */
  struct Data_t
  {
    double dt;
    int state; //state variable (State namespace)
    U_t u_in;  //controll input
    int clutch;
    int clutch_cmd;
    //
    double *current_positions; //motor current positions (deg)
    double *target_positions;  //motor target positions (deg)
    long *target_velocity;     //motor target velocity (rmp)
    //
    double wheel_vel;
    double *ev;
    double P, D; //gain

    Motors motor_params;
  };
}

static double angular_vel_to_steering_angle(double v, double omega)
{
  if (v == 0 || omega == 0)
    return 0; //zero radian

  double radius = v / omega;
  return atan((double)WHEEL_BASE / radius); //radian
}