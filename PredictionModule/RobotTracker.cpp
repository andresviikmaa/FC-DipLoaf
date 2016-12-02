#include "RobotTracker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"
#include "RobotLocation.h"
#include <iostream>

#ifdef SHOW_UI
extern IDisplay * display;
#endif // SHOW_UI

extern FieldState gFieldState;
extern FieldState gPartnerFieldState;
extern RobotState gRobotState;
extern RobotState gPartnerRobotState;

double getDistanceDiff(double initial, double diff) {
	//TODO:: probably wrong
	double dist1 = 125 * log(initial / 13.13);
	double dist2 = 125 * log(initial + diff / 13.13);
	return dist2 - dist1; 
}
RobotTracker::RobotTracker() : self(gFieldState.gates[BLUE_GATE], gFieldState.gates[YELLOW_GATE])
{
#ifdef SHOW_UI
	green = cv::imread("field.png", 1);   // Read the file
	field = cv::Mat(green.size(), CV_8UC3, cv::Scalar::all(245));
	c = cv::Point2d(green.size()) / 2;
#endif
	uchar id = 0;
	for (auto & ball : lastFieldState.balls){
		ball.id = ++id;
	}
}


RobotTracker::~RobotTracker()
{
}

void RobotTracker::Predict(double dt, bool mainCamUpdated, bool frontCamUpdated) 
{
	cv::Point3d movement = self.updateOdometer(gFieldState.self.wheelSpeeds, dt);

	DetectRobotLocation(dt);
	memcpy(&lastFieldState, &gFieldState, sizeof(FieldState));
	PredictLostBalls(movement, dt);
}
void RobotTracker::PredictLostBalls(cv::Point3d movement, double dt)
{
	for (auto &ball : gFieldState.balls){
		if (ball.isPredicted){
			ball.angle -= movement.z;
			assert(false);
			//ball.distance -= getDistanceDiffInverted(ball.distance, cv::norm(cv::Point2d(movement.x, movement.y)));
		}
	}
	

}
void RobotTracker::DetectRobotLocation(double dt){
	auto &Y = gFieldState.gates[YELLOW_GATE];
	auto &B = gFieldState.gates[BLUE_GATE];
	auto &lY = lastFieldState.gates[YELLOW_GATE];
	auto &lB = lastFieldState.gates[BLUE_GATE];

	if (Y.isValid && B.isValid){		
		self.updateFieldCoords(cv::Point2d(0, 0), dt);
	} else { 
		self.predict(dt);
	}


}
void RobotTracker::Reset(){
	auto &Y = gFieldState.gates[YELLOW_GATE];
	auto &B = gFieldState.gates[BLUE_GATE];

	if (Y.distance < B.distance){
		self.Reset(-300, -230, B.heading);
	}
	else {
		self.Reset(300, 230, Y.heading);
	}
}
#ifdef SHOW_UI
void RobotTracker::Draw(){
	green.copyTo(field);
	//cv::circle(field, gFieldState.self.rawFieldCoords + c, 24, cv::Scalar(0, 33, 255), 4);
	cv::circle(field, gFieldState.self.fieldCoords + c, 14, cv::Scalar(133, 33, 55), 4);
	cv::line(field, gFieldState.self.fieldCoords + c,
		cv::Point2d((40.0*sin(gFieldState.self.angle / 360 * TAU)), (-40 * cos(gFieldState.self.angle / 360 * TAU)))
		+ gFieldState.self.fieldCoords + c
		, cv::Scalar(133, 33, 55), 3);

	cv::circle(field, gFieldState.balls[gFieldState.closestBall].fieldCoords + c, 12, cv::Scalar(48, 154, 236), 2);
	for (size_t i = 0, ilen = MAX_BALLS; i < ilen; i++) {
		BallPosition &_ball = gFieldState.balls[i];
		cv::circle(field, _ball.fieldCoords + c, 7, cv::Scalar(48, 154, 236), -1);
		/*{
		message << "BAL " <<(int)balls[i].fieldCoords.x << " " << (int)balls[i].fieldCoords.y << " ";
		//SendMessage(message.str());
		}*/
	}


	if (!std::isnan(gFieldState.gates[BLUE_GATE].distance)) {
		cv::circle(field, gFieldState.gates[BLUE_GATE].fieldCoords + c, 14, cv::Scalar(236, 137, 48), 7);
		int r = (int)(gFieldState.gates[BLUE_GATE].polarMetricCoords.x);
		if (r < 0) r = 0;
		cv::circle(field, gFieldState.gates[BLUE_GATE].fieldCoords + c, r, cv::Scalar(236, 137, 48), 2);

		cv::line(field, gFieldState.self.fieldCoords + c,
			cv::Point2d((gFieldState.gates[BLUE_GATE].polarMetricCoords.x*sin((gFieldState.gates[BLUE_GATE].polarMetricCoords.y + gFieldState.self.angle) / 180 * CV_PI)),
			(-gFieldState.gates[BLUE_GATE].polarMetricCoords.x*cos((gFieldState.gates[BLUE_GATE].polarMetricCoords.y + gFieldState.self.angle) / 180 * CV_PI))
			) + gFieldState.self.fieldCoords + c
			, cv::Scalar(236, 137, 48), 3);
	}

	if (!std::isnan(gFieldState.gates[YELLOW_GATE].distance)) {
		cv::circle(field, gFieldState.gates[YELLOW_GATE].fieldCoords + c, 14, cv::Scalar(61, 255, 244), 7);
		int r = (int)(gFieldState.gates[YELLOW_GATE].polarMetricCoords.x);
		if (r < 0) r = 0;
		cv::circle(field, gFieldState.gates[YELLOW_GATE].fieldCoords + c, r, cv::Scalar(61, 255, 244), 2);

		cv::line(field, gFieldState.self.fieldCoords + c,
			cv::Point2d((gFieldState.gates[YELLOW_GATE].polarMetricCoords.x*sin((gFieldState.gates[YELLOW_GATE].polarMetricCoords.y + gFieldState.self.angle) / 360 * TAU)),
			(-gFieldState.gates[YELLOW_GATE].polarMetricCoords.x*cos((gFieldState.gates[YELLOW_GATE].polarMetricCoords.y + gFieldState.self.angle) / 360 * TAU))
			) + gFieldState.self.fieldCoords + c
			, cv::Scalar(61, 255, 244), 3);
	}

	display->ShowImage("Field", field, false);
}
#endif