#include "atv_driver/states/Travelling.h"

Travelling::Travelling()
{
}

Travelling::~Travelling()
{
}

int Travelling::_Transition(Definitions_t *data)
{
  if(abs(data->cmd_v) <= 0.001){
    return State::Braking;
  }

  return State::Travelling;
}
int Travelling::_Enter(Definitions_t *data)
{
  return 0;
}
int Travelling::_Process(Definitions_t *data)
{
  
  return 0;
}
int Travelling::_Exit(Definitions_t *data)
{
  return 0;
}