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
  data->motors.steer_pos.In = 0.0;
  data->motors.rear_pos.In = data->motors.rear_brake.Max;
  data->motors.front_pos.In = data->motors.front_brake.Max;
  data->motors.throttle_pos.In = data->motors.throttle.Min;

  return 0;
}
int Stop::_Process(soma_atv_driver::Data_t *data)
{
  data->motors.steer_pos.In = 0.0;
  data->motors.rear_pos.In = data->motors.rear_brake.Max;
  data->motors.front_pos.In = data->motors.front_brake.Max;
  data->motors.throttle_pos.In = data->motors.throttle.Min;

  return 0;
}
int Stop::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}