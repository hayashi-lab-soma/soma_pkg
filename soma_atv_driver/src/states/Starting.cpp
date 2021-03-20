#include "atv_driver/states/Starting.h"

Starting::Starting()
{
}

Starting::~Starting()
{
}

int Starting::_Transition(Definitions_t *data)
{
  if (abs(data->cmd_v) <= 0.001)
  {
    return State::Braking;
  }
  return State::Starting;
}
int Starting::_Enter(Definitions_t *data)
{
  if (data->cmd_v > 0.001)
  {
    data->clutch_cmd = Clutch::Forward;
  }
  else if (data->cmd_v < 0.001)
  {
    data->clutch_cmd = Clutch::Reverse;
  }

  // data->target_positions[0] =
  // data->target_velocity[1] =
  // data->target_velocity[3] =
<<<<<<< Updated upstream
=======
  data->target_positions[2] = Motor::FrontBrake::Min; //front brake
  data->target_positions[3] = 9.0;                    //throttle (deg)
>>>>>>> Stashed changes
}

int Starting::_Process(Definitions_t *data)
{
  if (data->clutch != data->clutch_cmd)
    return 0;

  //rear brake open (slowly)
  data->target_velocity[1] = 500; //rpm
  data->target_positions[1] = Motor::RearBrake::Min;
  data->target_positions[2] = Motor::FrontBrake::Min; //front brake
  data->target_positions[3] = 9.0; //throttle (deg)

  return 0;
}
int Starting::_Exit(Definitions_t *data)
{
  //change max velocity to default
  //data->
  //data
}