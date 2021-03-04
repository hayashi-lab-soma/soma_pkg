#ifndef STATEBASE_H
#define STATEBASE_H

#include <string>
#include <map>
#include "./soma_mse/definition.h"

class StateBase {
public:
  StateBase();
  virtual ~StateBase();

  int Transition(Data_t *data);
  int Enter(Data_t *data);
  int Process(Data_t *data);
  int Exit(Data_t *data);

protected:
  virtual int _Transition(Data_t *data) = 0;
  virtual int _Enter(Data_t *data) = 0;
  virtual int _Process(Data_t *data) = 0;
  virtual int _Exit(Data_t *data) = 0;
};

#endif
