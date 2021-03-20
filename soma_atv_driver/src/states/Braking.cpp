#include "atv_driver/states/Braking.h"

Braking::Braking()
{
}

Braking::~Braking()
{
}

int Braking::_Transition(Definitions_t *data)
{
  //if ATV velocity nearly equal 0, transition to State::Stop
  // return State::Braking;
  return State::Stop;
}
int Braking::_Enter(Definitions_t *data)
{
  data->target_positions[0] = 0.0;
  data->target_positions[1] = Motor::RearBrake::Max;
  data->target_positions[3] = 0.0;
}
int Braking::_Process(Definitions_t *data)
{
  data->target_positions[1] = Motor::RearBrake::Max;
}
int Braking::_Exit(Definitions_t *data)
{
  data->target_positions[2] = Motor::FrontBrake::Max;
}