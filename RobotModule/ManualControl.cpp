#include "ManualControl.h"

#define sign0(x) ((x > 0) - (x < 0))

ManualControl::ManualControl(ICommunicationModule *pComModule) :m_pComModule(pComModule)
{
}


ManualControl::~ManualControl()
{
}

void ManualControl::Step(double dt) {
	m_pComModule->Drive(speed, rotation);

	rotation -= sign0(rotation);
	speed.x -= sign0(acc.x)*dt;
	speed.y -= sign0(acc.y)*dt;
}