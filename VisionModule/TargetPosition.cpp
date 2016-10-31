#include "TargetPosition.h"


TargetPosition::~TargetPosition()
{
}


void TargetPosition::updateFieldCoords(cv::Point2d orgin) {

}

TargetPosition::TargetPosition(cv::Point orgin){
	fieldCoords.x = orgin.x;
	fieldCoords.y = orgin.y;
}

TargetPosition::TargetPosition(BallLocation orgin){
	fieldCoords = orgin.fieldCoords; 
	rawPixelCoords = orgin.rawPixelCoords;;
	polarMetricCoords = orgin.polarMetricCoords;
}
