#pragma once

#include "StateBase.h"

class Travelling : public StateBase
{
public:
  Travelling();
  ~Travelling();

  int _Transition(Definitions_t *data);
  int _Enter(Definitions_t *data);
  int _Process(Definitions_t *data);
  int _Exit(Definitions_t *data);
};