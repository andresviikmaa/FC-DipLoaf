#include "StateMachine.h"
#include <algorithm>

StateMachine::StateMachine(ISoccerRobot *pComModule,
	const std::map<DriveMode, DriveInstruction*> &driveModes) :driveModes(driveModes)
{
	m_pComModule = pComModule;
	for (auto driveMode : driveModes) driveMode.second->Init(pComModule);
	curDriveMode = this->driveModes.find(DRIVEMODE_IDLE);
	curDriveMode->second->onEnter();
}

void StateMachine::setTestMode(DriveMode mode){ testDriveMode = mode; }

void StateMachine::enableTestMode(bool enable)
{
	setTestMode(DRIVEMODE_IDLE);
	testMode = enable;
	if (!testMode) m_pComModule->Drive(0, 0, 0);
}

void StateMachine::Step(double dt) {
	DriveMode newMode = curDriveMode->first;
	newMode = testMode ? curDriveMode->second->step2(double(dt), newMode) : curDriveMode->second->step1(double(dt), newMode);
	auto old = curDriveMode;
	if (testMode) {
		if (testDriveMode != DRIVEMODE_IDLE && newMode == DRIVEMODE_IDLE) newMode = testDriveMode;
		else if (newMode != testDriveMode) {
			//newMode = DRIVEMODE_IDLE;
			//testDriveMode = DRIVEMODE_IDLE;
		}
	}

	if (newMode != curDriveMode->first) {
		//boost::mutex::scoped_lock lock(mutex);
		std::cout << "state: " << curDriveMode->second->name;

		curDriveMode->second->onExit();
		curDriveMode = driveModes.find(newMode);
		if (curDriveMode == driveModes.end()) {
			curDriveMode = driveModes.find(DRIVEMODE_IDLE);
			std::cout << "-> unknown state: " << newMode << std::endl;
		}
		std::cout << " -> " << curDriveMode->second->name << std::endl;

		curDriveMode->second->onEnter();
	}

}

std::string StateMachine::GetDebugInfo(){
	return curDriveMode->second->name;
}


StateMachine::~StateMachine()
{
	for (auto &mode : driveModes){ delete mode.second; }
}

DriveMode DriveInstruction::prevDriveMode = DRIVEMODE_IDLE;
DriveMode DriveInstruction::ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_IDLE;