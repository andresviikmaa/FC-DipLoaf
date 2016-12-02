#pragma once
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../CommonModule/Types.h"

#define sign0(x) ((x > 0) - (x < 0))

class IFieldState;
class ISoccerRobot;

typedef int DriveMode;
const DriveMode DRIVEMODE_CRASH = 0;
const DriveMode DRIVEMODE_BORDER_TO_CLOSE = 1;
const DriveMode DRIVEMODE_IDLE = 2;

#define STUCK_IN_STATE(delay) (boost::posix_time::microsec_clock::local_time() - actionStart).total_milliseconds() > delay


class DriveInstruction
{
protected:
	boost::posix_time::ptime actionStart;
	ISoccerRobot *m_pCom;
	Speed speed;
	static DriveMode prevDriveMode;
	static DriveMode ACTIVE_DRIVE_TO_BALL_MODE;
public:
	const std::string name;
	DriveInstruction(const std::string &name);
	void Init(ISoccerRobot *pCom);
	virtual void onEnter();
	virtual DriveMode step1(double dt, DriveMode driveMode);
	virtual DriveMode step2(double dt, DriveMode driveMode);
	virtual DriveMode step(double dt) = 0;
	virtual void onExit() {};

	const static bool USE_ANGLED_DRIVING = false;
	bool aimTarget(const ObjectPosition &target, Speed &speed, double errorMargin = (USE_ANGLED_DRIVING) ? 90 : 10);
	bool aimTargetAroundBall(const ObjectPosition &target, Speed &speed, double errorMargin);
	bool catchTarget(const ObjectPosition &target, Speed &speed);
	bool driveToTarget(const ObjectPosition &target, Speed &speed, double maxDistance = 50);
	bool driveToTargetWithAngle(const ObjectPosition &target, Speed &speed, double maxDistance = 50, double errorMargin = 10);
	bool preciseAim(const ObjectPosition &target, Speed &speed, double errorMargin = 5);
	bool preciseAim(const ObjectPosition &ball, const ObjectPosition &gate, Speed &speed, double errorMargin);
};
