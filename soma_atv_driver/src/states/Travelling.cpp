#include "atv_driver/states/Travelling.h"

/**
 * @brief Construct a new Travelling:: Travelling object
 * 
 */
Travelling::Travelling()
{
}
/**
 * @brief Destroy the Travelling:: Travelling object
 * 
 */
Travelling::~Travelling()
{
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Travelling::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) <= 0.001)
  {
    //if input STOP
    return State::Braking;
  }
  if ((data->clutch.out == Clutch::Forward && data->u_in.v < 0.0) || (data->clutch.out == Clutch::Reverse && data->u_in.v >= 0.0))
  {
    //if input move direction was changed ... it's dangerous situation!
    return State::Braking;
  }
  return State::Travelling;
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Travelling::_Enter(soma_atv_driver::Data_t *data)
{
  return 0;
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Travelling::_Process(soma_atv_driver::Data_t *data)
{
  //change steering angle
  data->motors.steer_pos.In = RAD2DEG(data->u_in.phi);

  // set throttle position by PD
  double UP = data->Kp * (data->ev[0] - data->ev[1]);
  // double UD = data->Kd / data->dt * (data->ev[0] - 2 * data->ev[1] + data->ev[2]);
  double UD = data->Kd * ((data->ev[0] - data->ev[1]) - (data->ev[1] - data->ev[2]));
  double M = UP + UD;                //deff operation value
  data->motors.throttle_pos.In += M; //add operation value
  data->motors.throttle_pos.In = std::max<double>(data->motors.throttle_pos.In, data->motors.throttle_regular);
  data->motors.throttle_pos.In = std::min<double>(data->motors.throttle_pos.In, data->motors.throttle.Max);

  // if wheel velocity is higher
  if (abs(data->wheel_vel) >= 1.5)
  {
    data->motors.rear_pos.In = data->motors.rear_brake.Max;
  }
  else{
    data->motors.rear_pos.In = data->motors.rear_brake.Min;
  }

  return 0;
}
int Travelling::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}