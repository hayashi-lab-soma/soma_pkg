#include "atv_driver/states/StateBase.h"

StateBase::StateBase() {}

StateBase::~StateBase() {}

//private base functions
int StateBase::Transition(Definitions_t *data) { return _Transition(data); }
int StateBase::Enter(Definitions_t *data) { return _Enter(data); }
int StateBase::Process(Definitions_t *data) { return _Process(data); }
int StateBase::Exit(Definitions_t *data) { return _Exit(data); }

// pure virtual functions
int StateBase::_Transition(Definitions_t *data) { return 0; }
int StateBase::_Enter(Definitions_t *data) { return 0; }
int StateBase::_Process(Definitions_t *data) { return 0; }
int StateBase::_Exit(Definitions_t *data) { return 0; }
