#include "atv_driver/states/StateBase.h"

StateBase::StateBase() {}

StateBase::~StateBase() {}

//private base functions
int StateBase::Transition(soma_atv_driver::Data_t *data) { return _Transition(data); }
int StateBase::Enter(soma_atv_driver::Data_t *data) { return _Enter(data); }
int StateBase::Process(soma_atv_driver::Data_t *data) { return _Process(data); }
int StateBase::Exit(soma_atv_driver::Data_t *data) { return _Exit(data); }

// pure virtual functions
int StateBase::_Transition(soma_atv_driver::Data_t *data) { return 0; }
int StateBase::_Enter(soma_atv_driver::Data_t *data) { return 0; }
int StateBase::_Process(soma_atv_driver::Data_t *data) { return 0; }
int StateBase::_Exit(soma_atv_driver::Data_t *data) { return 0; }
