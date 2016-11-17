#pragma once
#include "../CommonModule/Interfaces.h"
#include "refereeCom.h"
#include "../CommonModule/UdpServer.h"
#include "CoilBoard.h"

class ComModule :
	public ISoccerRobot, public UdpServer, public CoilBoard, public RefereeCom
{
protected:
	boost::asio::io_service &io;
	cv::Mat targetSpeedXYW = cv::Mat_<double>(3, 1);
	std::stringstream ss;
	bool ballInTribbler;
	int tribblerSpeed;

public:
	ComModule(boost::asio::io_service &io, const std::string ip_address, ushort port1, ushort port2=0);
	virtual ~ComModule();

	virtual void Drive(double fowardSpeed, double direction = 0, double angularSpeed = 0);
	virtual void Drive(const Speed &speed);
	virtual void Drive(const cv::Point2d &speed, double angularSpeed = 0);

	virtual void SetServoPos(int pos) {
		ss.clear();
		ss << "servos:" << pos << ":" << pos;
		SendMessage(ss.str());
	};
	virtual void Kick(int kick) {
		if (KickAllowed(kick)) {
			ss.clear();
			ss << "kick:" << kick;
			SendMessage(ss.str());
		}
	}
	virtual void ToggleTribbler(int speed){
		tribblerSpeed = speed;
	}
	virtual bool BallInTribbler(bool wait = false) {
		return CoilBoard::BallInTribbler(wait);
	}
	virtual long BallInTribblerTime() {
		return CoilBoard::BallInTribblerTime();
	};
	std::string GetDebugInfo() { return ""; }

	virtual void ProcessCommands();

	virtual bool MessageReceived(const std::string & message);
	virtual void SendMessages();
protected:

};

