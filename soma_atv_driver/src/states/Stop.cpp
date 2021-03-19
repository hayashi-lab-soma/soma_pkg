#include "atv_driver/states/Stop.h"

Stop::Stop()
{
}

Stop::~Stop()
{
}

int Stop::_Transition(Definitions_t *data)
{
  return State::Stop;
}
int Stop::_Enter(Definitions_t *data)
{

  data->target_positions[0] = 0.0;
  data->target_positions[1] = Motor::RearBrake::Max;
  data->target_positions[2] = Motor::FrontBrake::Max;
  data->target_positions[3] = Motor::Throttle::Min;

  return 0;
}
int Stop::_Process(Definitions_t *data)
{
  data->target_positions[0] = 0.0;
  data->target_positions[1] = Motor::RearBrake::Max;
  data->target_positions[2] = Motor::FrontBrake::Max;
  data->target_positions[3] = Motor::Throttle::Min;

  return 0;
}
int Stop::_Exit(Definitions_t *data)
{
  return 0;
}