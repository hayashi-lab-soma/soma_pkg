#ifndef MOVE_TO_H
#define MOVE_TO_H

#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //for transform pose
#include <dwa_local_planner/dwa_planner.h>
#include "./soma_mse/StateBase/StateBase.h"

class MoveTo : public StateBase
{
public:
  MoveTo(double _lim_d);
  ~MoveTo();

  int _Transition(Data_t *data);
  int _Enter(Data_t *data);
  int _Process(Data_t *data);
  int _Exit(Data_t *data);

private:
  double lim_d; // finish distance [m]
  geometry_msgs::PoseStamped fixed_target;
};

#endif
