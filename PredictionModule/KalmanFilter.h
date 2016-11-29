#pragma  once
#include <opencv2/opencv.hpp>

class KalmanFilter {
protected:
	cv::KalmanFilter KF = cv::KalmanFilter(4, 2, 0);
	cv::Mat_<double> measurement = cv::Mat_<double>(2, 1);
	cv::Mat_<double> estimated = cv::Mat_<double>(2, 1);
	int predictCount = 0;
public:
	KalmanFilter(const cv::Point2d &startPoint);
	cv::Point2d doFiltering(const cv::Point2d &point);
	cv::Point2d getPrediction();
	void resetFilter(const cv::Point2d &point);
	
};
