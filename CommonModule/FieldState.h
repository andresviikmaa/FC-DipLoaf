#pragma once
#include "Types.h"
const uchar MAX_BALLS = 15;

	// Filled by Vision Module
	struct FieldState {
		uint reserved = 0;
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
		cv::Point2d collisionRange; // which directions are blocked

		ObjectPosition partner;
		GatePosition gates[2]; //0 - BLUE_GATE, 1 -YELLOW_GATE
		ObjectPosition opponents[2];
		BallPosition balls[MAX_BALLS];

		/*
		//GatePosition partnerHomeGate;
		//virtual void SetTargetGate(OBJECT gate) = 0;
		//GatePosition targetGate;
		//GatePosition homeGate;
		*/
	};

