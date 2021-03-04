#include "soma_mse/StateBase/StateBase.h"

StateBase::StateBase() {}

StateBase::~StateBase() {}

int StateBase::Transition(Data_t *data) { return _Transition(data); }
int StateBase::Enter(Data_t *data) { return _Enter(data); }
int StateBase::Process(Data_t *data) { return _Process(data); }
int StateBase::Exit(Data_t *data) { return _Exit(data); }

// pure virtual functions
int StateBase::_Transition(Data_t *data) { return 0; }
int StateBase::_Enter(Data_t *data) { return 0; }
int StateBase::_Process(Data_t *data) { return 0; }
int StateBase::_Exit(Data_t *data) { return 0; }
