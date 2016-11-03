#pragma once
#include "../CommonModule/Types.h"
#include "ObjectPosition.h"
#include "TargetPosition.h"
#include "BallPosition.h"

class TargetPosition : public ObjectLocation
{
public:
	TargetPosition(){};
	TargetPosition(cv::Point orgin);
	TargetPosition(BallLocation orgin);
	virtual ~TargetPosition();
	void updateFieldCoords(cv::Point2d orgin);
};