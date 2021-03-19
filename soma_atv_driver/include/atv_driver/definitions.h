#pragma once

#include <string>
#include <map>

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

struct Definitions_t
{
  int state;
};
