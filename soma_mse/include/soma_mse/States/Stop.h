#ifndef STOP_H
#define STOP_H

#include "./soma_mse/StateBase/StateBase.h"

class Stop : public StateBase {
 public:
  Stop();

  int _Transition(Data_t *data);
  int _Enter(Data_t *data);
  int _Process(Data_t *data);
  int _Exit(Data_t *data);
};

#endif
