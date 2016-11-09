#pragma once
#include "Types.h"
#include <string>
#include <map>
#include <functional>
#include <opencv2/core.hpp>

//maybe use std::vector instead
typedef std::map<std::string, std::tuple<std::function<std::string()>, std::function<void()>>> SettingsList;

class ICamera
{
public:
	virtual cv::Mat & Capture(bool bFullFrame = false) = 0;
	virtual cv::Size GetFrameSize(bool bFullFrame = false) = 0;
	virtual double GetFPS() = 0;
	virtual cv::Mat & GetLastFrame(bool bFullFrame = false) = 0;
	virtual void TogglePlay() = 0;
	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name) = 0;
	virtual const std::string & getName() = 0;

};

class IConfigurableModule {
public:
	virtual SettingsList &GetSettings() = 0;
	virtual std::string GetDebugInfo() = 0;
};

class IUIEventListener {
public:
	// xy coordinates are from 0...1.0...
	virtual bool OnMouseEvent(int event, float x, float y, int flags, bool bMainArea) { return false; };
	virtual void OnKeyPress(char key) {};
};


class IDisplay {
public:
	virtual int createButton(const std::string& bar_name, char shortcut, std::function<void()> const &) = 0;
	virtual int Draw() = 0;
	virtual void clearButtons() = 0;
	virtual void ShowImage(const cv::Mat &image, bool main = true, bool flip = true) = 0;
	virtual void AddEventListener(IUIEventListener *pEventListener) = 0;
	virtual void RemoveEventListener(IUIEventListener *pEventListener) = 0;
	virtual void putText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) = 0;
	virtual void putShadowedText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) = 0;
	virtual void SwapDisplays() = 0;
	virtual void ToggleDisplay() = 0;
	virtual ~IDisplay() {};
};



class IVisionModule {
public:
	virtual const cv::Mat & GetFrame() = 0;
	virtual void PublishState() = 0;
	virtual ICamera * GetCamera() = 0;
	virtual void Enable(bool enable) = 0;
};

class IStateMachine {
public:
	virtual void Step(double dt) = 0;
	virtual void enableTestMode(bool enable) = 0;
	virtual std::string GetDebugInfo() = 0;
	virtual void Enable(bool enable) = 0;
	virtual void ProcessCommand(const std::string &command) {};
};

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
	virtual void Drive(const Speed &speed) = 0; /* forward, direction, rotation */
	virtual void Drive(const cv::Point2d &speed, double angularSpeed = 0) = 0;
	virtual bool BallInTribbler(bool wait = false) = 0;
	virtual long BallInTribblerTime() = 0;
	virtual void Kick(int force) = 0;
	virtual void ToggleTribbler(int speed) = 0;
	virtual std::string GetDebugInfo() = 0;
	virtual void ProcessCommands() = 0;
	//	virtual void SetRobotColor() = 0;
	//	virtual void SetGateColor() = 0;

};
