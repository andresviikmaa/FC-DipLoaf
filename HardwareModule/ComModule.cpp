#include "ComModule.h"
#include <boost/algorithm/string.hpp>
#include <thread>
#include <chrono>

extern cv::Mat wheelAngles;

ComModule::ComModule(boost::asio::io_service &io, const std::string ip_address, ushort port1, ushort port2):
	io(io), UdpServer(io, ip_address, port1, port2)
{


}


ComModule::~ComModule()
{
	for (int i = 0; i< 50; i++) {
		SendMessage("discharge");
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}



void ComModule::Drive(double fowardSpeed, double direction, double angularSpeed) {

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

void ComModule::Drive(const cv::Point2d &speed, double angularSpeed) {
	targetSpeedXYW.at<double>(0) = speed.x;
	targetSpeedXYW.at<double>(1) = speed.y;
	targetSpeedXYW.at<double>(2) = angularSpeed;
}


void ComModule::ProcessCommands() {
	io.poll();
}

bool ComModule::MessageReceived(const std::string & message) {
	if (message.empty()) return false;
	if (*message.begin() != '<' && *message.rbegin() != '>') return false;
	std::vector<std::string> params;
	boost::split(params, message.substr(1, message.size() - 1), boost::is_any_of(":"));
	const auto &command = params[0];
	if (command == "speeds" /*<speeds:%d:%d:%d:%d:%d>*/) {

	}else if (message == "ref" /*<ref:%s>*/) {
		handleMessage(params[1]);
	}
	else if (message == "toggle-side" /*<toggle-side>*/) {

	}
	else if (message == "toggle-go" /*<toggle-go>*/) {

	}
	else if (message == "ball" /*<ball:%d>*/) {
		bool ball = false;
		SetBallInTribbler(params[1][0]=='1');
	}
	else if (message == "<adc:%.1f>") {

	}
	return true;
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