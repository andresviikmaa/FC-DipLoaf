#pragma once
#include <opencv2/core.hpp>

class RobotFinder
{
public:
	RobotFinder();
	virtual ~RobotFinder();
	virtual bool Locate(cv::Mat &threshHoldedImage, cv::Mat &frameHSV, cv::Mat &frameBGR, std::vector<cv::Point2i> &objectCoords);
};

