#pragma once
//#include "DistanceCalibrator.h"
#include <boost/property_tree/ptree.hpp>
#include <atomic>
#include <opencv2/core.hpp>
class DistanceCalculator{
protected:
	double getDistance(const cv::Point2d &pos, const cv::Point2d &orgin) const;
public:
	DistanceCalculator();
	double getDistanceInverted(const cv::Point2d &pos, const cv::Point2d &orgin) const;
	cv::Point2d getPolarCoordinates(const cv::Point2d &pos, const cv::Point2d &orgin) const;
	cv::Point2d getFieldCoordinates(const cv::Point2d &pos, const cv::Point2d &orgin) const;
	void loadConf();

	static double angleBetween(const cv::Point2d &a, const cv::Point2d &b) {
		double alpha = atan2(a.y, a.x) - atan2(b.y, b.x);
		double alphaDeg = alpha * 180. / CV_PI;
		if (alphaDeg < 0) alphaDeg += 360;
		return alphaDeg;
	}
	static bool angleInRange(cv::Point2d point, cv::Point2d range);

private:
	boost::property_tree::ptree pt;
	std::vector<int> realDistances;
	std::vector<double> references;
	std::atomic_bool m_bEnabled;
};