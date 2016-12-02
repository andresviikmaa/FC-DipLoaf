#include "DriveInstruction.h"

#include "../CommonModule/Interfaces.h"
#include "../HardwareModule/HardwareInterfaces.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"

extern RobotState gRobotState;

DriveInstruction::DriveInstruction(const std::string &name) : name(name) {
};

void DriveInstruction::Init(ISoccerRobot *pCom) {
	m_pCom = pCom;
};
void DriveInstruction::onEnter() {
	actionStart = boost::posix_time::microsec_clock::local_time();
};
DriveMode DriveInstruction::step1(double dt, DriveMode driveMode) {
	//not executed in test mode
	if (gRobotState.gameMode == GAME_MODE_END_HALF && driveMode != DRIVEMODE_IDLE) {
		std::cout << "Stoping game (referee stop)" << std::endl;
		m_pCom->ToggleTribbler(0);
		return DRIVEMODE_IDLE;
	}
	/*
	// handle crash
	if (m_pFieldState->collisionWithBorder && driveMode != DRIVEMODE_BORDER_TO_CLOSE){
	prevDriveMode = driveMode;
	std::cout << "To close to border" << std::endl;
	return DRIVEMODE_BORDER_TO_CLOSE;
	}
	*/
	/*
	if (m_pFieldState->collisionWithUnknown && driveMode != DRIVEMODE_CRASH){
	std::cout << "Crash" << std::endl;
	return DRIVEMODE_CRASH;
	}
	*/
	// recover from crash
	/*
	if (!m_pFieldState->collisionWithBorder && driveMode == DRIVEMODE_BORDER_TO_CLOSE){
	DriveMode tmp = prevDriveMode;
	prevDriveMode = DRIVEMODE_IDLE;
	//std::cout <<"aaa" <<std::endl;
	return tmp;
	}
	*/
	/*
	if (!m_pFieldState->collisionWithUnknown && driveMode == DRIVEMODE_CRASH){
	DriveMode tmp = prevDriveMode;
	prevDriveMode = DRIVEMODE_IDLE;
	//std::cout << "bbb" << std::endl;
	return tmp;
	}
	*/
	prevDriveMode = driveMode;
	speed = { 0, 0, 0 };
	return step(dt);
};
DriveMode DriveInstruction::step2(double dt, DriveMode driveMode) {
	speed = { 0, 0, 0 };
	return step(dt);
}

bool DriveInstruction::aimTarget(const ObjectPosition &target, Speed &speed, double errorMargin) {
	double heading = target.heading;
	if (fabs(heading) > errorMargin) {
		speed.rotation = -sign0(heading) * std::min(40.0, std::max(fabs(heading), 5.0));

		speed.velocity = 0;
		speed.heading = 0;
		return false;
	}
	else return fabs(heading) < errorMargin;
}

bool DriveInstruction::aimTargetAroundBall(const ObjectPosition &target, Speed &speed, double errorMargin) {
	double heading = target.heading;
	if (fabs(heading) > errorMargin) {
		speed.rotation = (heading) ;// turn toward target
		speed.velocity = fabs(heading) * 2;//side moving speed same as rotation
		speed.heading = -sign0(heading) * 90; //move sideways opposite to turning
	}
	return fabs(heading) <= errorMargin;
}

bool DriveInstruction::catchTarget(const ObjectPosition &target, Speed &speed) {
	if (m_pCom->BallInTribbler()) return true;
	double heading = target.heading;
	double dist = target.distance;
	speed.velocity = 60;
	speed.heading = 0;
	speed.rotation = 0;
	return false;
}

bool DriveInstruction::driveToTarget(const ObjectPosition &target, Speed &speed, double maxDistance) {
	double dist = target.distance;
	if (dist > maxDistance) {
		speed.velocity = std::max(20.0, dist);
		return false;
	}
	else {
		speed.velocity = 20;
		return true;
	}
}

bool DriveInstruction::driveToTargetWithAngle(const ObjectPosition &target, Speed &speed, double maxDistance, double errorMargin) {
	double heading = target.heading;
	double dist = target.distance;
	double velocity = 0;
	double direction = 0;
	double angularSpeed = 0;
	bool onPoint = false;
	if (fabs(heading) > errorMargin) {
		if (dist > maxDistance) {
			velocity = std::max(30.0, dist); //max speed is limited to 190 in wheelController.cpp 
			direction = heading; //drive towards target
			angularSpeed = sign0(heading) * 20; //meanwhile rotate slowly to face the target
		}
		else { //at location but facing wrong way
			angularSpeed = sign0(heading) * std::max(fabs(heading) * 0.5, 10.0); //rotate
		}
	}
	else {
		if (dist > maxDistance) {
			velocity = std::max(30.0, dist);
			direction = heading; //drive towards target
		}
		else onPoint = true;
	}
	speed.velocity = velocity;
	speed.heading = direction;
	speed.rotation = -angularSpeed;
	return onPoint;
}

bool DriveInstruction::preciseAim(const ObjectPosition &target, Speed &speed, double errorMargin) {
	if (target.distance < 150 && fabs(target.heading) > errorMargin){

		speed.heading = target.heading * 1.5;//if target close move sideways
		speed.velocity = 15;
	}
	else if (fabs(target.heading) > errorMargin){
		speed.rotation = -target.heading/2;
		speed.velocity = 40;
	}
	return fabs(target.heading) < errorMargin;
}

bool DriveInstruction::preciseAim(const ObjectPosition &ball, const ObjectPosition &gate, Speed &speed, double errorMargin) {

	speed.heading = ball.heading * 1.4;
	speed.rotation = -gate.heading / 1.5;
	speed.velocity = 60;

	return fabs(ball.heading - gate.heading) < 1.8;
}



