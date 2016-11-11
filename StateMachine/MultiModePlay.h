#pragma once
#include "StateMachine.h"

class MultiModePlay :
	public StateMachine
{
private:
	bool isMaster;
public:
	MultiModePlay(ISoccerRobot *pComModule, bool bMaster);
	~MultiModePlay();
};

