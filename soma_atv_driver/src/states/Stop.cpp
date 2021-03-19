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
}
int Stop::_Process(Definitions_t *data)
{
}
int Stop::_Exit(Definitions_t *data)
{
}