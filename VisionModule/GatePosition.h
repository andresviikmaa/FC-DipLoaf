#pragma once
#include "../CommonModule/Types.h"
#include "ObjectPosition.h"

class GateLocation : public ObjectLocation
{
public:
	GateLocation(){};
	GateLocation(OBJECT gate);
	virtual ~GateLocation();
	virtual void updateFieldCoords(cv::Point2d orgin = cv::Point2d(0, 0)){};
	cv::Point2d minCornerPolarCoords;
};
