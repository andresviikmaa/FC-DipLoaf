#pragma once
#include "Types.h"
enum RunMode : uchar{
	ROBOT_MODE_IDLE = 0,
	ROBOT_MODE_1VS1,
	ROBOT_MODE_2VS2
};
struct RobotState
{
	uint reserved = 1;
	uint stateSize;
	OBJECT ourTeam = TEAM_PINK;
	OBJECT oppoonentTeam = TEAM_PURPLE;
	OBJECT targetGate = YELLOW_GATE;
	OBJECT homeGate = BLUE_GATE;

	char FIELD_MARKER = 'A';
	char TEAM_MARKER = 'A';
	char ROBOT_MARKER = 'A';

	RunMode runMode = ROBOT_MODE_IDLE; // 1vs1, idle, 2vs2
	uchar gameMode;  // referee commands
	uchar pendingGameMode;
	uchar playState; // statemachine state
};

