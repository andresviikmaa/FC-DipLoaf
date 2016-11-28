#pragma once
#include <opencv2/core.hpp>
#include "../CommonModule/FieldState.h"
class RobotTracker
{
public:
	RobotTracker();
	~RobotTracker();
	void Predict(double dt, bool mainCamUpdated, bool frontCamUpdated);
	void DetectRobotLocation();
#ifdef SHOW_UI
	void Draw();
#endif
protected:
	cv::Mat green;
	cv::Mat field;// = cv::Mat(310, 500, CV_8UC3, cv::Scalar::all(245)); // blink display
	cv::Point2d c/*enter*/;
	FieldState lastFieldState;
	int ballLost1 = 0;
	int ballLost2 = 0;
};

