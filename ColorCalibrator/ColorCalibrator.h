#pragma once
#include <opencv2/core.hpp>
#include "../VisionModule/DummyVision.h"

class ColorCalibrator: public DummyVision {
protected:
	cv::Mat image;
public:
	ColorCalibrator();
	virtual void ProcessFrame(cv::Mat &image);
	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name);
	virtual ~ColorCalibrator();

};