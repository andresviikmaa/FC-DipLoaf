#pragma once
#include "C:\personal\github\FC-DipLoaf\CommonModule\Interfaces.h"
class ManualControl :
	public IStateMachine
{
protected:
	ICommunicationModule *m_pComModule;
	// accelerations
	cv::Point2d acc = {0 ,0};
	cv::Point2d speed = { 0 ,0 };
	double rotation = 0;
public:
	ManualControl(ICommunicationModule *pComModule);
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
};

