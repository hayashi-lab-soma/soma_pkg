#include "atv_driver/states/Braking.h"

Braking::Braking()
{
}

Braking::~Braking()
{
}

int Braking::_Transition(Definitions_t *data)
{
  return State::Braking;
}
int Braking::_Enter(Definitions_t *data)
{
}
int Braking::_Process(Definitions_t *data)
{
}
int Braking::_Exit(Definitions_t *data)
{
}