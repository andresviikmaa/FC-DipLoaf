#pragma once
#include <opencv2/core.hpp>
#include "../VisionModule/DummyVision.h"

class ColorCalibrator {
protected:
	cv::Mat image;
public:
	ColorCalibrator();
	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name);
	virtual ~ColorCalibrator();

};