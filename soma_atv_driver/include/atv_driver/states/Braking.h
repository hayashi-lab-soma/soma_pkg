#pragma once

#include "StateBase.h"

class Braking : public StateBase
{
public:
  Braking();
  ~Braking();

  int _Transition(Definitions_t *data);
  int _Enter(Definitions_t *data);
  int _Process(Definitions_t *data);
  int _Exit(Definitions_t *data);
};