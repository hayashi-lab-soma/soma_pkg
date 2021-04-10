#include "atv_driver/states/Starting.h"

/**
 * @brief Construct a new Starting:: Starting object
 * 
 */
Starting::Starting()
{
  T = 0.0;
}

/**
 * @brief Destroy the Starting:: Starting object
 * 
 */
Starting::~Starting()
{
}

/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Starting::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) <= 0.001) //
  {
    return State::Braking;
  }
  if (abs(data->wheel_vel) >= 0.1) //completed starting
  {
    return State::Travelling;
  }
  return State::Starting;
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Starting::_Enter(soma_atv_driver::Data_t *data)
{
  if (data->u_in.v > 0.001)
  {
    data->clutch_cmd = Clutch::Forward;
  }
  else if (data->u_in.v < -0.001)
  {
    data->clutch_cmd = Clutch::Reverse;
  }

  //rear brake open (slowly)
  data->target_velocity[1] = (long)data->motor_params.rear_brake_starting_state_low_rpm; //change max rpm of rear brake
  data->target_positions[1] = data->motor_params.rear_brake.Min;                         //open rear brake
  data->target_positions[2] = data->motor_params.front_brake.Min;                        //open front brake
  data->target_positions[3] = data->motor_params.throttle_regular;                       //set throttle (deg)
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Starting::_Process(soma_atv_driver::Data_t *data)
{
  //wait for clutch state changed
  if (data->clutch != data->clutch_cmd)
    return 0;

  data->target_positions[0] = RAD2DEG(data->u_in.phi);             //degrees
  data->target_positions[1] = data->motor_params.rear_brake.Min;   //open rear brake
  data->target_positions[2] = data->motor_params.front_brake.Min;  //open front brake
  data->target_positions[3] = data->motor_params.throttle_regular; //set throttle (deg)

  T += data->dt; //spent time measurement

  if (T >= 1.0)
  {
    if (abs(data->wheel_vel) <= 0.1)
    {
      //increase throttle position
      data->target_positions[3] = data->current_positions[3] + 0.1;
    }
  }
  return 0;
}
/**
 * @brief 
 * 
 * @param data 
 * @return int 
 */
int Starting::_Exit(soma_atv_driver::Data_t *data)
{
  //change max velocity to default
  data->target_velocity[1] = 3500;

  T = 0.0;
}