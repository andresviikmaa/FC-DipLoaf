#pragma once
#include "../CommonModule/Types.h"
#include "ObjectPosition.h"
#include "KalmanFilter.h"
#include <boost/timer/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <atomic>
class BallLocation : public ObjectLocation
{
public:
	BallLocation() :useKalman(false){};
	BallLocation(bool useKalman) :useKalman(useKalman){};
	BallLocation(const BallLocation & ballPos){
		rawPixelCoords = ballPos.rawPixelCoords;
		polarMetricCoords = ballPos.polarMetricCoords;
		fieldCoords.x = ballPos.fieldCoords.x;
		fieldCoords.y = ballPos.fieldCoords.y;
	};
	virtual ~BallLocation();
	int id;
	bool isValid;
	/*std::atomic_*/bool isUpdated;
	void setIsUpdated(bool updated);
	void updateFieldCoords(cv::Point2d orgin = cv::Point2d(0, 0), double heading = 0);
	// for simulator
	double speed = 0;
	double heading = 0;
	void filterCoords(const BallLocation &ball, bool reset=false);
	void predictCoords();
	cv::Point2i lastRawCoords;
private:
	KalmanFilter filter = KalmanFilter(cv::Point2i(0, 0));
	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime lastStep = time;
	bool useKalman = false;
};


