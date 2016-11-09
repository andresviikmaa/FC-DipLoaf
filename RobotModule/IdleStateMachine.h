#pragma once
#include "C:\personal\github\FC-DipLoaf\CommonModule\Interfaces.h"
class IdleStateMachine :
	public IStateMachine
{
public:
	IdleStateMachine(ICommunicationModule *pComModule);
	~IdleStateMachine();
	virtual void Step(double dt);
	virtual void enableTestMode(bool enable) {}
	virtual std::string GetDebugInfo() {}
};

