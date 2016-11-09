#pragma once
#include "Types.h"
extern "C" {
	class FieldState {
	public:
		uchar reserved = 0;
		uint stateSize;

		//group 4  together
		GameMode gameMode;
		RobotColor robotColor = ROBOT_COLOR_BLUE_UP;
		bool isPlaying;
		bool collisionWithBorder;

		bool collisionWithUnknown;
		bool obstacleNearBall;
		bool gateObstructed;
		OBJECT targetGate;

		OBJECT homeGate;
		uchar ballCount; // number or balls visible
		uchar closestBall; // index to closeset ball by distance
		uchar closestBallInFront;
		ObjectPosition partner;
		RobotPosition self; //Robot distance on field
		GatePosition gates[2]; //0 - BLUE_GATE, 1 -YELLOW_GATE
		ObjectPosition opponents[2];
		cv::Point2d collisionRange; // which directions are blocked
		BallPosition balls[15];

		/*
		//GatePosition partnerHomeGate;
		//virtual void SetTargetGate(OBJECT gate) = 0;
		//GatePosition targetGate;
		//GatePosition homeGate;
		*/
	};
}