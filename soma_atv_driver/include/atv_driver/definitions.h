#pragma once

#include <string>
#include <map>
#include <math.h>

#define DEG2RAD(x) (x/180.0*M_PI)
#define RAD2DEG(x) (x/M_PI*180.0)

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

namespace Mode
{
  const int Forward = 1;
  const int Backward = 2;
  const int Stop = 3;

  const std::map<int, std::string> Str = {{Forward, "Forward"},
                                          {Backward, "Backward"},
                                          {Stop, "Stop"}};

}

namespace Motor
{
  namespace Steering
  {
    const double Min = -25.0;
    const double Max = 25.0;
  }
  namespace RearBrake
  {
    const double Min = 0.0;
    const double Max = 9.0;
  }
  namespace FrontBrake
  {
    const double Min = 0.0;
    const double Max = 14.0;
  }
  namespace Throttle
  {
    const double Min = 0.0;
    const double Max = 20.0;
  }

}

namespace Clutch
{
  const int Forward = 1;
  const int Reverse = 2;
  const int Free = 3;

  const std::map<int, std::string> Str{{Forward, "Forward"},
                                       {Reverse, "Reverse"},
                                       {Free, "Free"}};
}

struct Definitions_t
{
  int state;
  int cmd;
  //commands
  double cmd_v;
  //
  double *current_positions; //motor current positions (deg)
  double *target_positions;  //motor target positions (deg)
  long *target_velocity; //motor target velocity (rmp)
  //
  int clutch;
  int clutch_cmd;
};
