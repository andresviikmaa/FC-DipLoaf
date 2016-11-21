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
	OBJECT ourTeam = TEAM1;
	OBJECT oppoonentTeam = TEAM2;
	OBJECT targetGate = YELLOW_GATE;
	OBJECT homeGate = BLUE_GATE;

	char FIELD_MARKER = 'A';
	char TEAM_MARKER = 'A';
	char ROBOT_MARKER = 'A';

	RunMode runMode = ROBOT_MODE_IDLE;
	GameMode gameMode = GAME_MODE_END_HALF;

};

