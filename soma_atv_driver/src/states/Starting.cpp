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
  if (abs(data->u_in.v) <= 0.001) //if input STOP
  {
    return State::Braking;
  }
  if ((data->clutch.out == Clutch::Forward && data->u_in.v < 0.0) || (data->clutch.out == Clutch::Reverse && data->u_in.v >= 0.0))
  {
    //if input move direction was changed ... it's dangerous situation!
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
    data->clutch.in = Clutch::Forward;
  }
  else if (data->u_in.v < -0.001)
  {
    data->clutch.in = Clutch::Reverse;
  }

  //rear brake open (slowly)
  data->motors.rear_vel.In = (long)data->motors.rear_brake_starting_state_low_rpm; //change max rpm of rear brake
  data->motors.rear_pos.In = data->motors.rear_brake.Min;                          //open rear brake
  //front open
  data->motors.front_pos.In = data->motors.front_brake.Min; //open front brake
  //throttle to regular
  data->motors.throttle_pos.In = data->motors.throttle_regular; //set throttle (deg)

  T = 0.0;
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
  if (data->clutch.out != data->clutch.in)
    return 0;

  data->motors.steer_pos.In = RAD2DEG(data->u_in.phi);          //degrees
  data->motors.rear_pos.In = data->motors.rear_brake.Min;       //open rear brake
  data->motors.front_pos.In = data->motors.front_brake.Min;     //open front brake
  data->motors.throttle_pos.In = data->motors.throttle_regular; //set throttle (deg)

  T += data->dt; //spent time measurement

  if (T >= 2.0)
  {
    if (abs(data->wheel_vel) <= 0.1)
    {
      //increase throttle position
      data->motors.throttle_pos.In = data->motors.throttle_pos.Out + 0.3;
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
  data->motors.rear_vel.In = 3500;

  T = 0.0;
}