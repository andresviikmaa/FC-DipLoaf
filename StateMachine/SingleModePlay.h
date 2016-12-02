#include "StateMachine.h"

enum SingleModeDriveStates {
	//DRIVEMODE_IDLE = 0,
	DRIVEMODE_DRIVE_TO_BALL = DRIVEMODE_IDLE + 1,
	DRIVEMODE_DRIVE_HOME,
	DRIVEMODE_DRIVE_TO_BALL_FRONT,
	DRIVEMODE_RECOVER_CRASH,
	DRIVEMODE_EXIT,

};
class SingleModeIdle : public Idle {
	virtual void onEnter();

	virtual DriveMode step(double dt);
};

class DriveToBall : public DriveInstruction
{
public:
	virtual void onEnter();
	DriveToBall(const std::string &name = "DRIVE_TO_BALL") : DriveInstruction(name){};
	virtual DriveMode step(double dt);

private:
	bool toggledDribbler = false;
	ObjectPosition initialBall;
	ObjectPosition initialGate;
};

class SingleModePlay : public StateMachine {
public:

	SingleModePlay(ISoccerRobot *pComModule);

};