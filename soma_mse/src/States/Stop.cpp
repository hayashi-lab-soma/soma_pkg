#include "./soma_mse/States/Stop.h"

Stop::Stop() {}

int Stop::_Transition(Data_t *data)
{
  if (data->command == Command::MoveTo)
    return State::MoveTo;

  if (data->command == Command::GoHome)
    return State::GoHome;

  return State::Stop;
}

int Stop::_Enter(Data_t *data)
{
  data->u_t.linear.x = 0.0;
  data->u_t.angular.z = 0.0;
  return 0;
}

int Stop::_Process(Data_t *data)
{
  data->u_t.linear.x = 0.0;
  data->u_t.angular.z = 0.0;
  return 0;
}

int Stop::_Exit(Data_t *data) { return 0; }
