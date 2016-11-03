#pragma once
#include "Types.h"

class FieldState {
public:
	GameMode gameMode;
	bool isPlaying;
	RobotColor robotColor = ROBOT_COLOR_BLUE_UP;
	bool collisionWithBorder;
	bool collisionWithUnknown;
	bool obstacleNearBall;
	struct {
		double x;
		double y;
	} collisionRange; // which directions are blocked
	ushort ballCount; // number or balls visible
	ushort closestBall; // index to closeset ball by distance
	ushort closestBallInFront;
	BallPosition balls[14];
	GatePosition gates[3]; //0 - unused 1 - BLUE_GATE, 2 -YELLOW_GATE
	GatePosition yellowGate;
	RobotPosition self; //Robot distance on field
	ObjectPosition partner;
	ObjectPosition opponents[2];
	//GatePosition partnerHomeGate;
	bool gateObstructed;
	//virtual void SetTargetGate(OBJECT gate) = 0;
	//GatePosition targetGate;
	//GatePosition homeGate;
	OBJECT targetGate;
	OBJECT homeGate;
};
