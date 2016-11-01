#include "ObjectPosition.h"
#include "../CommonModule/DistanceCalculator.h"



void ObjectLocation::updateCoordinates(const cv::Point2d &pixelCoords, const cv::Point2d &polarCoords) {
	lastFieldCoords = fieldCoords;
	rawPixelCoords = pixelCoords;
	polarMetricCoords = polarCoords;
}