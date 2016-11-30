#include "ComModule.h"
#include <boost/algorithm/string.hpp>
#include <thread>
#include <chrono>
#include "../CommonModule/FieldState.h"
#include "opencv2/imgproc.hpp"

extern cv::Mat wheelAngles;
extern FieldState gFieldState;

ComModule::ComModule(boost::asio::io_service &io, const std::string ip_address, ushort port1, ushort port2):
	io(io), UdpServer(io, ip_address, port1, port2)
{


}

ComModule::ComModule(boost::asio::io_service &io, const std::string ip_address, ushort port1):
	io(io), UdpServer(io, ip_address, port1)
{
	//SendMessage("fs:0");
	SendMessage("charge");
}

ComModule::~ComModule()
{
	//SendMessage("fs:1");
	//for (int i = 0; i< 10; i++) {
	//	SendMessage("discharge");
	//	std::this_thread::sleep_for(std::chrono::milliseconds(300));
	//}
	//std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ComModule::sendAck(const std::string & message){

	SendMessage("ref:" + message);
};


void ComModule::Drive(double fowardSpeed, double direction, double angularSpeed) {
	//direction *= -1.;
	//angularSpeed *= -1.;
	gFieldState.self.distance = fowardSpeed;
	gFieldState.self.heading = direction;
	gFieldState.self.angle = angularSpeed;

	const int maxSpeed = 60;
	/*
	direction = 45;
	angularSpeed = 0;
	fowardSpeed = 30;
	*/
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


	targetSpeedXYW.at<double>(0) = cos((direction)* CV_PI / 180.0)* fowardSpeed;
	targetSpeedXYW.at<double>(1) = sin((direction)* CV_PI / 180.0)* fowardSpeed;
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
	std::string tmp = message.substr(1, message.size() - 1);
	boost::split(params, tmp, boost::is_any_of(":"));
	const auto &command = params[0];
	const int SIMULATOR_SPEED = 1;
	if (command == "speeds" && params.size() > 4/*<speeds:%d:%d:%d:%d:%d>*/) {
		//std::cout << "cmd: " << tmp << std::endl;
		gFieldState.self.wheelSpeeds[0] = atoi(params[1].c_str());
		gFieldState.self.wheelSpeeds[1] = atoi(params[2].c_str());
		gFieldState.self.wheelSpeeds[2] = atoi(params[3].c_str());
		gFieldState.self.wheelSpeeds[3] = atoi(params[4].c_str());
	}
	else if (command == "ref" /*<ref:%s>*/) {
		std::cout << "cmd: " << tmp << std::endl;
		handleMessage(params[1]);
	}
	else if (command == "toggle-side" /*<toggle-side>*/) {

	}
	else if (command == "toggle-go" /*<toggle-go>*/) {

	}
	else if (command == "ball" /*<ball:%d>*/) {
		std::cout << "cmd: " << tmp << std::endl;
		bool ball = false;
		SetBallInTribbler(params[1][0]=='1');
	}
	else if (command == "<adc:%.1f>") {

	}
	return true;
}
void ComModule::SendMessages() {
	std::stringstream ss;
	ss.clear();
	ss << "speeds";

	cv::Mat speeds = wheelAngles * targetSpeedXYW *8;
	ss << ":" << (int)speeds.at<double>(2);
	ss << ":" << -(int)speeds.at<double>(3);
	ss << ":" << -(int)speeds.at<double>(0);
	ss << ":" << (int)speeds.at<double>(1);
	ss << ":" << -tribblerSpeed*30;

	std::string tmp = ss.str();
	SendMessage(tmp);
}