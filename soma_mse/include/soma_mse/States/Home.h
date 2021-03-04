#pragma once

#include "./soma_mse/StateBase/StateBase.h"

class Home : public StateBase {
 public:
  Home(double _lim_d);
  ~Home() {}

  int _Transition(Data_t *data);
  int _Enter(Data_t *data);
  int _Process(Data_t *data);
  int _Exit(Data_t *data);

 private:
  double lim_d;  // finish distance [m]
  // PurePursuit::Config PpCfg;
};

