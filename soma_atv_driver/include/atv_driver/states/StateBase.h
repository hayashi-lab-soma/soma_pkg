#pragma once

#include <string>
#include <map>
#include "atv_driver/definitions.h"

class StateBase {
public:
  StateBase();
  virtual ~StateBase();

  int Transition(Definitions_t *data);
  int Enter(Definitions_t *data);
  int Process(Definitions_t *data);
  int Exit(Definitions_t *data);

protected:
  virtual int _Transition(Definitions_t *data) = 0;
  virtual int _Enter(Definitions_t *data) = 0;
  virtual int _Process(Definitions_t *data) = 0;
  virtual int _Exit(Definitions_t *data) = 0;
};
