#include "atv_driver/states/Starting.h"

Starting::Starting()
{
}

Starting::~Starting()
{
}

int Starting::_Transition(Definitions_t *data)
{
  return State::Starting;
}
int Starting::_Enter(Definitions_t *data)
{
}
int Starting::_Process(Definitions_t *data)
{
}
int Starting::_Exit(Definitions_t *data)
{
}