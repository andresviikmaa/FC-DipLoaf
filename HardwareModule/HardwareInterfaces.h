#pragma once
#include "../CommonModule/Types.h"
#include <string>
//#include <opencv2/core.hpp>

class ISerialListener {
public:
	virtual void DataReceived(const std::string & message) = 0;
};
class ISerial : public ISerialListener {
public:
	virtual void SendCommand(int id, const std::string &cmd, int param = INT_MAX) = 0;
	virtual void WriteString(const std::string &s) = 0;
	virtual void SetMessageHandler(ISerialListener* callback) {};
};

class ICommunicationModule {
public:
	virtual void Drive(double fowardSpeed, double direction = 0, double angularSpeed = 0) = 0;
	// needed for spinAroundDribbler https://github.com/kallaspriit/soccervision/blob/80840c921ad0935ed2e0718ed405613af3e51aa1/src/Robot.cpp#L385
	virtual void Drive(const Speed &speed) = 0; /* x,y speed components */
	virtual bool BallInTribbler(bool wait = false) = 0;
	virtual long BallInTribblerTime() = 0;
	virtual void Kick(int force) = 0;
	virtual void ToggleTribbler(int speed) = 0;
	virtual std::string GetDebugInfo() = 0;

};