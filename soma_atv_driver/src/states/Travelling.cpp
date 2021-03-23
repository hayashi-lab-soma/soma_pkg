#include "atv_driver/states/Travelling.h"

Travelling::Travelling()
{
}

Travelling::~Travelling()
{
}

int Travelling::_Transition(soma_atv_driver::Data_t *data)
{
  if (abs(data->u_in.v) <= 0.001)
  {
    return State::Braking;
  }

  return State::Travelling;
}
int Travelling::_Enter(soma_atv_driver::Data_t *data)
{
  return 0;
}
int Travelling::_Process(soma_atv_driver::Data_t *data)
{
  data->target_positions[0] = RAD2DEG(data->u_in.phi);//degrees


  // set throttle position by PD
	//calclate gains
	double Pout = data->P * (data->ev[0] - data->ev[1]);
	double Dout = data->D / data->dt  * (data->ev[0] - 2 * data->ev[1] + data->ev[2]);

	// qInfo() << "------------------------------------------------";
	// qInfo() << "velo:" << data->v[0] << "v_ref:" << data->V_ref;
	// qInfo() << "ev:" << data->ev[0] << "v_err:" << data->V_err;
	// qInfo() << "KP:" << data->P << "," << "Pout:" << data->Pout;
	// qInfo() << "KD:" << data->D << "," << "Dout:" << data->Dout;

	double delta_acc = Pout + Dout;
	// qInfo() << "Delta Acc[deg]:" << delta_acc;

	data->target_positions[3] += delta_acc;
	// qInfo() << "acc[deg]:" << acc;

  data->target_positions[3] = std::max<double>(data->target_positions[3], data->throttle_offset);
  data->target_positions[3] = std::min<double>(data->target_positions[3], data->throttle_max);

  return 0;
}
int Travelling::_Exit(soma_atv_driver::Data_t *data)
{
  return 0;
}