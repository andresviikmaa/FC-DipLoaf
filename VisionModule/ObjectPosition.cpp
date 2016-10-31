#include "ObjectPosition.h"
#include "../CommonModule/DistanceCalculator.h"
extern DistanceCalculator gDistanceCalculator;



void ObjectLocation::updateRawCoordinates(const cv::Point2d pos, cv::Point2d orgin) {
	lastFieldCoords = fieldCoords;
	rawPixelCoords = pos;
	polarMetricCoords = gDistanceCalculator.getPolarCoordinates(orgin, pos);
}