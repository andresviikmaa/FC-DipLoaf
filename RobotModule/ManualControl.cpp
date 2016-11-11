#include "ManualControl.h"
#include <sstream>

#define sign0(x) ((x > 0) - (x < 0))

ManualControl::ManualControl(ISoccerRobot *pComModule) :m_pComModule(pComModule)
{
}


ManualControl::~ManualControl()
{
}

void ManualControl::Step(double dt) {
	m_pComModule->Drive(speed, rotation);

	// deaccelerate
	rotation -= sign0(rotation);
	speed.x -= sign0(speed.x)*dt; 
	speed.y -= sign0(speed.y)*dt;
}

void ManualControl::ProcessCommand(const std::string &command) {
	std::stringstream ss(command);
	char key;
	ss >> key;
	switch (key)
	{
		case 'R':
			rotation -= 10;
			break;
		case 'L':
			rotation += 10;
			break;
		case 'A':
			speed.x -= 10;
			break;
		case 'D':
			speed.x += 10;
			break;
		case 'W':
			speed.y += 10;
			break;
		case 'S':
			speed.y -= 10;
			break;
		case 'Q':
			speed = cv::Point2d(0, 0); 
			rotation = 0;
			break;
		case 'K':
			m_pComModule->Kick(2500);
			break;
		case 'Z':
			m_pComModule->ToggleTribbler(100);
			break;
		case 'X':
			m_pComModule->ToggleTribbler(0);
			break;
		default:
			break;
	}
};