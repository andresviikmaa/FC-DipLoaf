#include "SingleModePlay.h"
#define ANDRESE_CATCH_BALL
#include "../CommonModule/Types.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"
extern FieldState gFieldState;
extern RobotState gRobotState;

enum {
	DRIVEMODE_DRIVE_TO_BALL_NAIVE = 5000,
	DRIVEMODE_DIRVE_TO_BALL_AVOID_TURN,
	DRIVEMODE_DRIVE_TO_BALL_ANGLED,
	DRIVEMODE_DRIVE_TO_BALL_AIM_GATE,
	DRIVEMODE_ROTATE_AROUND_BALL
};

void SingleModeIdle::onEnter() {
	m_pCom->ToggleTribbler(0);
	m_pCom->Drive(0, 0, 0);
}

DriveMode SingleModeIdle::step(double dt) {
	if (gRobotState.gameMode == GAME_MODE_START_PLAY) {
		gRobotState.gameMode = GAME_MODE_IN_PROGRESS;
	}
	if (gRobotState.gameMode != GAME_MODE_IN_PROGRESS) {
		return DRIVEMODE_IDLE;
	}
	return  DRIVEMODE_DRIVE_TO_BALL; 
}

/*BEGIN DriveToBall*/
void DriveToBall::onEnter()
{
	DriveInstruction::onEnter();		
	initialBall = gFieldState.balls[gFieldState.closestBall];
	initialGate = gFieldState.gates[gRobotState.targetGate];
	if (ACTIVE_DRIVE_TO_BALL_MODE == DRIVEMODE_IDLE)
		ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
}

DriveMode DriveToBall::step(double dt){
	if(initialBall.distance == 0) return DRIVEMODE_DRIVE_HOME;
	return ACTIVE_DRIVE_TO_BALL_MODE;
}
class DriveToBallFront : public DriveToBall
{
public:
	DriveToBallFront(const std::string &name = "DRIVE_TO_BALL_FRONT") : DriveToBall(name){};
	DriveMode step(double dt)
	{

		auto &target = gFieldState.ballsFront[gFieldState.closestBallTribbler];
		if (target.distance > 10000) {
			return DRIVEMODE_DRIVE_TO_BALL;//target too far	
		}
		if (preciseAim(target, gFieldState.gates[gRobotState.targetGate], speed, 2)) {
			if (m_pCom->BallInTribbler()) {
				m_pCom->Drive(30, 0, 0);
				m_pCom->Kick(gFieldState.gates[gRobotState.targetGate].distance > 350 ? 5000 : 2500);
				m_pCom->Drive(0, 0, 0);
			}
		}
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_DRIVE_TO_BALL_FRONT;
	}
};

class DriveToBallAimGate : public DriveInstruction
{
public:
	DriveToBallAimGate(const std::string &name = "DRIVE_TO_BALL_AIM_GATE") : DriveInstruction(name){};

	DriveMode step(double dt){		
		if (m_pCom->BallInTribbler() || gFieldState.ballsFront[gFieldState.closestBallTribbler].distance < 10000){ //if frontCam sees ball
			m_pCom->Drive(0, 0, 0);
			return DRIVEMODE_DRIVE_TO_BALL_FRONT;
		}
		const ObjectPosition &ball = gFieldState.balls[gFieldState.closestBall];
		const ObjectPosition &gate = gFieldState.gates[gRobotState.targetGate];
		double gateHeading = gate.heading;
		double ballHeading = ball.heading;
		double ballDistance = ball.distance;
		double rotation = 0;
		double errorMargin = 5;
		double maxDistance = 40;
		if (ballDistance > 10000) return DRIVEMODE_DRIVE_HOME;
		if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		double heading = 0;
		double speed = 0;
		if (ballDistance > maxDistance) {
			heading = ballHeading;
			if (fabs(heading) > 30) heading = sign0(heading)*(fabs(heading) + 15);
			speed = std::max(60.0, ballDistance);
		}
		else {
			if (fabs(ballHeading) <= errorMargin && fabs(gateHeading) <= errorMargin){
				m_pCom->Drive(0, 0, 0);
				return DRIVEMODE_DRIVE_TO_BALL_FRONT;
			}
			if (fabs(ballHeading) > errorMargin){
				heading = ballHeading + sign0(ballHeading) * 60;
				speed = 70;
			}
			rotation = 0;
			if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		}
		m_pCom->Drive(speed, heading, rotation);
		return DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
	}
};
class DriveToHome : public DriveInstruction
{
public:
	DriveToHome(const std::string &name = "DRIVE_HOME") : DriveInstruction(name){};
	virtual DriveMode step(double dt){
		if (gFieldState.closestBall < MAX_BALLS-1) return DRIVEMODE_DRIVE_TO_BALL;
		auto target = gFieldState.gates[gRobotState.homeGate];
		if (target.distance < 50) return DRIVEMODE_DRIVE_TO_BALL;
		else m_pCom->Drive(40, target.heading);
	return DRIVEMODE_DRIVE_HOME;
	}
};

std::pair<DriveMode, DriveInstruction*> SingleDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new SingleModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_HOME, new DriveToHome()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL, new DriveToBall()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_FRONT, new DriveToBallFront()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate()),
};

SingleModePlay::SingleModePlay(ISoccerRobot *pComModule)
		:StateMachine(pComModule, TDriveModes(SingleDriveModes, SingleDriveModes + sizeof(SingleDriveModes) / sizeof(SingleDriveModes[0]))){};

