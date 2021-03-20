#include "atv_driver/states/Travelling.h"

Travelling::Travelling()
{
}

Travelling::~Travelling()
{
}

int Travelling::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) <= 0.001)
  {
    return State::Braking;
  }

  return State::Travelling;
}
int Travelling::_Enter(soma_atv_driver::Data_t *data)
{
  return 0;
}
int Travelling::_Process(soma_atv_driver::Data_t *data)
{

  return 0;
}
int Travelling::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}