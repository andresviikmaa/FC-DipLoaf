#pragma once
#include "ObjectFinder.h"
//#include "FieldState.h"
//#include "types.h"
#include "VisionInterfaces.h"
class BallFinder
{
public:
	BallFinder();
	virtual ~BallFinder();
	virtual bool Locate(cv::Mat &threshHoldedImage, cv::Mat &frameHSV, cv::Mat &frameBGR, std::vector<cv::Point2d> &objectCoords);
//	void populateBalls(ThresholdedImages &HSVRanges, cv::Mat &frameHSV, cv::Mat &frameBGR, OBJECT target, FieldState *pFieldState);
	static bool validateBall(ThresholdedImages &HSVRanges, cv::Point endPoint, cv::Mat &frameHSV, cv::Mat &frameBGR);
};

