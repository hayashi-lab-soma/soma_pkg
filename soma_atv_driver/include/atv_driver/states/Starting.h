#pragma once

#include "StateBase.h"

class Starting : public StateBase
{
public:
  Starting();
  ~Starting();

  int _Transition(Definitions_t *data);
  int _Enter(Definitions_t *data);
  int _Process(Definitions_t *data);
  int _Exit(Definitions_t *data);
};