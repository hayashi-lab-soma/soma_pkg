#include "atv_driver/states/Starting.h"

Starting::Starting()
{
}

Starting::~Starting()
{
}

int Starting::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) <= 0.001)
  {
    return State::Braking;
  }
  if (abs(data->wheel_vel) >= 0.05)
  {
    return State::Travelling;
  }
  return State::Starting;
}
int Starting::_Enter(soma_atv_driver::Data_t *data)
{
  if (data->u_in.v > 0.001)
  {
    data->clutch_cmd = Clutch::Forward;
  }
  else if (data->u_in.v < 0.001)
  {
    data->clutch_cmd = Clutch::Reverse;
  }

  //steering

  //rear brake open (slowly)
  data->target_velocity[1] = data->rear_brake_slow_open_rpm;       //rpm
  data->target_positions[2] = data->motors_poslim.front_brake.Min; //open front brake
}

int Starting::_Process(soma_atv_driver::Data_t *data)
{
  if (data->clutch != data->clutch_cmd)
    return 0;

  data->target_positions[0] = RAD2DEG(data->u_in.phi);             //degrees
  data->target_positions[1] = data->motors_poslim.rear_brake.Min;  //open rear brake
  data->target_positions[2] = data->motors_poslim.front_brake.Min; //open front brake
  data->target_positions[3] = data->throttle_offset;               //set throttle (deg)

  return 0;
}
int Starting::_Exit(soma_atv_driver::Data_t *data)
{
  //change max velocity to default
  data->target_velocity[1] = 3500;
  //data
}