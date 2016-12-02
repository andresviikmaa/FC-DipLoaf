#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const cv::Point2d &startPoint){
	KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1);

	/*
	measurement(0) = (float)(startPoint.x);
	measurement(1) = (float)(startPoint.y);
	estimated(0) = (float)(startPoint.x);
	estimated(1) = (float)(startPoint.y);
	*/
	measurement.setTo(cv::Scalar(0));

	KF.statePre.at<float>(0) = (float)(startPoint.x);
	KF.statePre.at<float>(1) = (float)(startPoint.y);
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	/*
	KF.statePost.at<float>(0) = (float)(startPoint.x);
	KF.statePost.at<float>(1) = (float)(startPoint.y);
	KF.statePost.at<float>(2) = (float)(startPoint.x);
	KF.statePost.at<float>(3) = (float)(startPoint.y);
	*/

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-6));
	setIdentity(KF.errorCovPost, cv::Scalar::all(1e-6));
}

cv::Point2d KalmanFilter::doFiltering(const cv::Point2d &point) {
	// First predict, to update the internal statePre variable
	getPrediction();

	//Get point
	measurement(0) = point.x;
	measurement(1) = point.y;

	//The update phase
	estimated = KF.correct(measurement);
	cv::Point2d statePt(estimated.at<double>(0), estimated.at<double>(1));
	return statePt;
//	return cv::Point2d(estimated);
  
}

cv::Point2d KalmanFilter::getPrediction() {
	cv::Mat prediction = KF.predict();
	return cv::Point2d(prediction.at<double>(0), prediction.at<double>(1));
	//return cv::Point2d(prediction);
}


void KalmanFilter::resetFilter(const cv::Point2d &startPoint){
	measurement(0) = startPoint.x;
	measurement(1) = startPoint.y;
	estimated(0) = startPoint.x;
	estimated(1) = startPoint.y;

	KF.statePre.at<float>(0) = (float)(startPoint.x);
	KF.statePre.at<float>(1) = (float)(startPoint.y);
	KF.statePre.at<float>(2) = (float)(startPoint.x);
	KF.statePre.at<float>(3) = (float)(startPoint.y);

	KF.statePost.at<float>(0) = (float)(startPoint.x);
	KF.statePost.at<float>(1) = (float)(startPoint.y);
	KF.statePost.at<float>(2) = (float)(startPoint.x);
	KF.statePost.at<float>(3) = (float)(startPoint.y);
}

