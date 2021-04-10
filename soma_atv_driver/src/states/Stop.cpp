#include "atv_driver/states/Stop.h"

Stop::Stop()
{
}

Stop::~Stop()
{
}

int Stop::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) > 0.001)
  {
    return State::Starting;
  }
  return State::Stop;
}
int Stop::_Enter(soma_atv_driver::Data_t *data)
{
  data->target_positions[0] = 0.0;
  data->target_positions[1] = data->motors_poslim.rear_brake.Max;
  data->target_positions[2] = data->motors_poslim.front_brake.Max;
  data->target_positions[3] = data->motors_poslim.throttle.Min;

  return 0;
}
int Stop::_Process(soma_atv_driver::Data_t *data)
{
  data->target_positions[0] = 0.0;
  data->target_positions[1] = data->motors_poslim.rear_brake.Max;
  data->target_positions[2] = data->motors_poslim.front_brake.Max;
  data->target_positions[3] = data->motors_poslim.throttle.Min;

  return 0;
}
int Stop::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}