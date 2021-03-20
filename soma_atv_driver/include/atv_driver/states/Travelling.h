#pragma once

#include "StateBase.h"

class Travelling : public StateBase
{
public:
  Travelling();
  ~Travelling();

  int _Transition(soma_atv_driver::Data_t *data);
  int _Enter(soma_atv_driver::Data_t *data);
  int _Process(soma_atv_driver::Data_t *data);
  int _Exit(soma_atv_driver::Data_t *data);
};