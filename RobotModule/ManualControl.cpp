#include "ManualControl.h"
#include <sstream>

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

void ManualControl::ProcessCommand(const std::string &command) {
	std::stringstream ss(command);
	char key;
	ss >> key;
	switch (key)
	{
		case 'r':
			rotation -= 10;
			break;
		case 'l':
			rotation += 10;
			break;
		case 'a':
			speed.x -= 10;
			break;
		case 'd':
			speed.x += 10;
			break;
		case 'w':
			speed.y += 10;
			break;
		case 's':
			speed.y -= 10;
			break;
		case 'q':
			speed = cv::Point2d(0, 0); 
			rotation = 0;
			break;
		case 'k':
			m_pComModule->Kick(2500);
			break;
		case 'z':
			m_pComModule->ToggleTribbler(100);
			break;
		case 'x':
			m_pComModule->ToggleTribbler(0);
			break;
		default:
			break;
	}
};