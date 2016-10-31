#pragma once
#include "../CommonModule/CalibrationConfReader.h"
#include <opencv2/core.hpp>

class ColorCalibrator : public CalibrationConfReader {
protected:
	cv::Mat image;
public:
	ColorCalibrator();
	virtual void LoadImage(cv::Mat &image);
	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name);
	virtual ~ColorCalibrator();

};