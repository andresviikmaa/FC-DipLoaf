#pragma once
#include "Types.h"

typedef std::vector<BallPosition> BallArray;
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
					  //BallPosition balls[number_of_balls]; //All others are distance from self and heading to it
	BallArray balls;
	BallPosition closestBall;
	GatePosition blueGate;
	GatePosition yellowGate;
	RobotPosition self; //Robot distance on field
	ObjectPosition partner;
	BallArray opponents;
	GatePosition partnerHomeGate;
	bool gateObstructed;
	//virtual void SetTargetGate(OBJECT gate) = 0;
	GatePosition targetGate;
	GatePosition homeGate;
};
