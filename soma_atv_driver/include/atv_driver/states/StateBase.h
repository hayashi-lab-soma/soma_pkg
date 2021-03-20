#pragma once

#include <string>
#include <map>
#include "atv_driver/definitions.h"

class StateBase {
public:
  StateBase();
  virtual ~StateBase();

  int Transition(soma_atv_driver::Data_t *data);
  int Enter(soma_atv_driver::Data_t *data);
  int Process(soma_atv_driver::Data_t *data);
  int Exit(soma_atv_driver::Data_t *data);

protected:
  virtual int _Transition(soma_atv_driver::Data_t *data) = 0;
  virtual int _Enter(soma_atv_driver::Data_t *data) = 0;
  virtual int _Process(soma_atv_driver::Data_t *data) = 0;
  virtual int _Exit(soma_atv_driver::Data_t *data) = 0;
};
