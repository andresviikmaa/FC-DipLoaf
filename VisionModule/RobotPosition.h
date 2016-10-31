#pragma once
#include "../CommonModule/Types.h"
#include "ObjectPosition.h"
#include "GatePosition.h"
#include "KalmanFilter.h"
class RobotLocation : public ObjectLocation
{
public:
	RobotLocation(GateLocation &yellowGate, GateLocation &blueGate, cv::Point initialCoords = cv::Point(0,0));
	virtual ~RobotLocation();
	virtual void updatePolarCoords();
	void updateFieldCoordsNew(cv::Point2d orgin, double dt);
	void updateFieldCoords(cv::Point2d orgin, double dt);
	double getAngle();
	cv::Point2d rawFieldCoords; // (x, y) Coordinates to display objects on field by, relative to field
	void predict(double dt);
private:
	GateLocation & yellowGate, & blueGate; // use references that point somewhere
	void initPolarCoordinates();
	std::pair<cv::Point, cv::Point> intersectionOfTwoCircles(cv::Point circle1center, double circle1Rad, cv::Point circle2center, double circle2Rad);
	bool isRobotAboveCenterLine(double yellowGoalAngle, double blueGoalAngle);
	double getRobotDirection();
	KalmanFilter filter;
	double tmp;
	double lastRotation = 0;
	double rotationSpeed = 0;
};
