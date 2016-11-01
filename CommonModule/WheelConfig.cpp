#include <opencv2/core.hpp>
#include "WheelConfig.h"
cv::Mat wheelAngles = (cv::Mat_<double>(4, 3) <<
	-sin(45.0 / 180 * CV_PI), cos(45.0 / 180 * CV_PI), 1,
	-sin(135.0 / 180 * CV_PI), cos(135.0 / 180 * CV_PI), 1,
	-sin(225.0 / 180 * CV_PI), cos(225.0 / 180 * CV_PI), 1,
	-sin(315.0 / 180 * CV_PI), cos(315.0 / 180 * CV_PI), 1);
