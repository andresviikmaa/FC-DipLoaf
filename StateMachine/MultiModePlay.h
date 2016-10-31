#pragma once
#include "StateMachine.h"

class MultiModePlay :
	public StateMachine
{
private:
	bool isMaster;
public:
	MultiModePlay(ICommunicationModule *pComModule, bool bMaster);
	~MultiModePlay();
};

