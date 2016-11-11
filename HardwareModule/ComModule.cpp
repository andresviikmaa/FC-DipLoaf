#include "ComModule.h"

extern cv::Mat wheelAngles;

ComModule::ComModule(boost::asio::io_service &io, const std::string ip_address, ushort port1, ushort port2 = 0):
	io(io), UdpServer(io, ip_address, port1, port2)
{


}


ComModule::~ComModule()
{
	SendMessage("discharge")
}



void ComModule::Drive(double fowardSpeed, double direction = 0, double angularSpeed = 0) {

	const int maxSpeed = 190;
	if (fowardSpeed > maxSpeed) fowardSpeed = maxSpeed;
	if (fowardSpeed < -maxSpeed) fowardSpeed = -maxSpeed;

	if ((abs(angularSpeed) + fowardSpeed) > 190) {
		if (angularSpeed > 0) {
			fowardSpeed = angularSpeed - angularSpeed;
		}
		else {
			fowardSpeed = angularSpeed + angularSpeed;
		}

	}


	targetSpeedXYW.at<double>(0) = sin(direction* CV_PI / 180.0)* fowardSpeed;
	targetSpeedXYW.at<double>(1) = cos(direction* CV_PI / 180.0)* fowardSpeed;
	targetSpeedXYW.at<double>(2) = angularSpeed;

};

void ComModule::Drive(const Speed &speed) {
	Drive(speed.velocity, speed.heading, speed.rotation);
};

void ComModule::Drive(const cv::Point2d &speed, double angularSpeed = 0) {
	targetSpeedXYW.at<double>(0) = speed.x;
	targetSpeedXYW.at<double>(1) = speed.y;
	targetSpeedXYW.at<double>(2) = angularSpeed;
}


void ComModule::ProcessCommands() {
	io.poll();
}

void ComModule::MessageReceived(const std::string & message) {
	if (message == "<speeds:%d:%d:%d:%d:%d>") {

	}else if (message == "<ref:%s>") {

	}
	else if (message == "speeds") {

	}
	else if (message == "<toggle-side>") {

	}
	else if (message == "<toggle-go>") {

	}
	else if (message == "<ball:%d>") {

	}
	else if (message == "<adc:%.1f>") {

	}
	
}
void ComModule::SendMessages() {
	ss.clear();
	ss << "speeds";

	cv::Mat speeds = wheelAngles * targetSpeedXYW;
	for (auto i = 0; i < speeds.rows; i++) {
		ss << ":" << (int)speeds.at<double>(i);
	}

	SendMessage(ss.str());
}