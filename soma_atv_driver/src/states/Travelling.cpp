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
  if ((data->clutch==Clutch::Forward && data->u_in.v < 0.0)
  || (data->clutch==Clutch::Reverse && data->u_in.v >= 0.0))
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
  data->target_positions[0] = RAD2DEG(data->u_in.phi);

  // set throttle position by PD
  double UP = data->Kp * (data->ev[0] - data->ev[1]);
  // double UD = data->Kd / data->dt * (data->ev[0] - 2 * data->ev[1] + data->ev[2]);
  double UD = data->Kd * ((data->ev[0] - data->ev[1]) - (data->ev[1] - data->ev[2]));
  double M = UP + UD;             //deff operation value
  data->target_positions[3] = data->target_positions[3] + M; //add operation value
  data->target_positions[3] = std::max<double>(data->target_positions[3], data->motor_params.throttle_regular);
  data->target_positions[3] = std::min<double>(data->target_positions[3], data->motor_params.throttle.Max);

  return 0;
}
int Travelling::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}