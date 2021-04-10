#include "atv_driver/states/Braking.h"

Braking::Braking()
{
}

Braking::~Braking()
{
}

int Braking::_Transition(soma_atv_driver::Data_t *data)
{
  //if ATV velocity nearly equal 0, transition to State::Stop
  if (abs(data->wheel_vel) <= 0.01)
  {
    return State::Stop;
  }
  return State::Braking;
}
int Braking::_Enter(soma_atv_driver::Data_t *data)
{
  data->target_positions[0] = 0.0;
  data->target_positions[1] = data->motors_poslim.rear_brake.Max;
  data->target_positions[3] = 0.0;
}
int Braking::_Process(soma_atv_driver::Data_t *data)
{
  data->target_positions[1] = data->motors_poslim.rear_brake.Max;
}
int Braking::_Exit(soma_atv_driver::Data_t *data)
{
  data->target_positions[2] = data->motors_poslim.rear_brake.Max;
}