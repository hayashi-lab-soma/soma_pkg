#include "atv_driver/states/Starting.h"

Starting::Starting()
{
}

Starting::~Starting()
{
}

int Starting::_Transition(Definitions_t *data)
{
  switch (data->cmd)
  {
  case Mode::Stop:
    return State::Braking;
  }
  return State::Starting;
}
int Starting::_Enter(Definitions_t *data)
{
  if (data->cmd == Mode::Forward)
  {
    data->clutch_cmd = Clutch::Forward;
  }
  if (data->cmd == Mode::Backward)
  {
    data->clutch_cmd = Clutch::Reverse;
  }

  // data->target_positions[0] =
  // data->target_velocity[1] =
  // data->target_velocity[3] =
}
int Starting::_Process(Definitions_t *data)
{
  if (data->clutch != data->clutch_cmd)
    return 0;

  // data->
  //

  return 0;
}
int Starting::_Exit(Definitions_t *data)
{
  //change max velocity to default
  //data->
  //data
}