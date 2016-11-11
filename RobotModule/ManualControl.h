#pragma once
#include "../CommonModule/Interfaces.h"
class ManualControl :
	public IStateMachine
{
protected:
	ISoccerRobot *m_pComModule;
	// accelerations
	cv::Point2d acc = {0 ,0};
	cv::Point2d speed = { 0 ,0 };
	double rotation = 0;
public:
	ManualControl(ISoccerRobot *pComModule);
	~ManualControl();
	virtual void Step(double dt);
	virtual void enableTestMode(bool enable) {}
	virtual std::string GetDebugInfo() {
		return "";
	}
	virtual void Enable(bool enable) {
		speed = { 0, 0 };
		rotation = 0;
	
	};
	virtual void ProcessCommand(const std::string &command);
};

