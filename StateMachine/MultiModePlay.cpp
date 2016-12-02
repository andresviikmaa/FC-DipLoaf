#include "MultiModePlay.h"
#include "SingleModePlay.h"
#include "../CommonModule/Types.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"

const float KICKOFF_ANGLE = 45.;
extern FieldState gFieldState;
extern RobotState gRobotState;
extern RobotState gPartnerRobotState;

#define sign(x) ((x > 0) ? 1 : -1)

enum MultiModeDriveStates {
	//2v2 modes
	DRIVEMODE_2V2_DEFENSIVE = 100,
	DRIVEMODE_2V2_CATCH_KICKOFF,
	DRIVEMODE_2V2_DRIVE_HOME,
	DRIVEMODE_2V2_GOAL_KEEPER,
	DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE,
	DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER,

};

class MasterModeIdle : public Idle {

	virtual DriveMode step(double dt) {
		switch (gRobotState.gameMode) {

		case GAME_MODE_START_OPPONENT_KICK_OFF:
		case GAME_MODE_START_OPPONENT_FREE_KICK:
		case GAME_MODE_START_OPPONENT_PENALTY:
			return DRIVEMODE_2V2_DEFENSIVE;

		case GAME_MODE_START_OUR_FREE_KICK:
		case GAME_MODE_START_OUR_KICK_OFF:
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER;

		case GAME_MODE_START_OUR_PENALTY:
		case GAME_MODE_START_OUR_INDIRECT_FREE_KICK:
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;

		}
		return DRIVEMODE_IDLE;
	}
};

class SlaveModeIdle : public Idle {
	virtual DriveMode step(double dt) {
		switch (gRobotState.gameMode) {
		case GAME_MODE_START_OPPONENT_FREE_KICK:
		case GAME_MODE_START_OPPONENT_KICK_OFF:
		case GAME_MODE_START_OUR_PENALTY:
		case GAME_MODE_START_OUR_INDIRECT_FREE_KICK:
			return DRIVEMODE_2V2_DEFENSIVE;

		case GAME_MODE_START_OUR_FREE_KICK:
		case GAME_MODE_START_OUR_KICK_OFF:
			return DRIVEMODE_2V2_CATCH_KICKOFF;
		}
		return DRIVEMODE_IDLE;
	}
};

//========= DRIVEMODES ===============//

class Defensive : public DriveInstruction
{
public:
	Defensive() : DriveInstruction("2V2_DEFENSIVE"){};
	virtual DriveMode step(double dt){
		auto & target = gFieldState.partner;
		if (gPartnerRobotState.driveState != DRIVEMODE_2V2_GOAL_KEEPER && gPartnerRobotState.driveState != DRIVEMODE_2V2_DRIVE_HOME){ return DRIVEMODE_2V2_DRIVE_HOME; }//ToDo goalKeeper message 
		else{
			auto &ballFront = gFieldState.ballsFront[gFieldState.closestBallTribbler];
			auto &ball = gFieldState.balls[gFieldState.closestBall];
			if (ball.distance < 10000 || ballFront.distance < 10000)  { //see ball - go for ball
				return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
			}
			auto homeGate = gFieldState.gates[gRobotState.homeGate];
			double homeGateDist = homeGate.distance;
			double gateAngle = homeGate.heading - 180 * sign0(homeGate.heading);
			aimTarget(target, speed, 2);
			if (homeGateDist < 130 || homeGate.minCornerPolarCoords.x < 50) {
				driveToTargetWithAngle(target, speed, 40, 5);
				speed.velocity = 50;
			}
			else {
				if (gateAngle < 0) {
					speed.heading = 90;
				}
				else {
					speed.heading = -90;
				}
				speed.velocity = 50;
			}			
		}
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_2V2_DEFENSIVE;
	}
};

class CatchKickOff : public DriveInstruction
{
private:
	bool active = false;
public:
	CatchKickOff() : DriveInstruction("2V2_CATCH_KICKOFF"){};

	virtual DriveMode step(double dt){
		if (gPartnerRobotState.driveState != DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER && gPartnerRobotState.driveState != DRIVEMODE_IDLE) {
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
		}

		return DRIVEMODE_2V2_CATCH_KICKOFF;
	}
};

class DriveHome2v2 : public DriveInstruction
{
public:
	DriveHome2v2() : DriveInstruction("2V2_DRIVE_HOME"){};
	virtual DriveMode step(double dt){
		ObjectPosition &lastGatePosition = gFieldState.gates[gRobotState.homeGate];
		if (DriveInstruction::driveToTargetWithAngle(lastGatePosition, speed, 65))return DRIVEMODE_2V2_GOAL_KEEPER;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_2V2_DRIVE_HOME;
	}
};

class GoalKeeper : public DriveInstruction
{
private:
	bool wentLeft = false;
public:
	GoalKeeper() : DriveInstruction("2V2_GOAL_KEEPER"){};

	virtual DriveMode step(double dt){
		
		auto ball = gFieldState.ballsFront[gFieldState.closestBallTribbler];
		if (ball.distance > 10000) {
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;//target too far	
		}
		auto homeGate = gFieldState.gates[gRobotState.homeGate];
		double homeGateDist = homeGate.distance;
		double gateAngle = homeGate.heading - 180 * sign0(homeGate.heading);
		aimTarget(gFieldState.gates[gRobotState.targetGate], speed, 2);
		if (homeGateDist < 30 || homeGate.minCornerPolarCoords.x < 30) {
			speed.velocity = 30;
		} else {	
			if (gateAngle < 0) {
				speed.heading = 90;
			}
			else {
				speed.heading = -90;
			}
			speed.velocity = 50;
		}

		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_2V2_GOAL_KEEPER;
	}
};

class DriveToBallAimGate2v2 : public DriveInstruction
{
public:
	DriveToBallAimGate2v2(const std::string &name = "2V2_DRIVE_TO_BALL_AIM_GATE") : DriveInstruction(name) {};

	DriveMode step(double dt) {
		if (m_pCom->BallInTribbler() || gFieldState.ballsFront[gFieldState.closestBallTribbler].distance < 10000){ //if frontCam sees ball
						
			auto &target = gFieldState.ballsFront[gFieldState.closestBallTribbler];
			if (preciseAim(target, gFieldState.gates[gRobotState.targetGate], speed, 2)) {
				if (m_pCom->BallInTribbler())
					m_pCom->Kick(5000);
			}
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
		}

		const ObjectPosition &ball = /*gFieldState.closestBallInFront != MAX_BALLS - 1 ? gFieldState.balls[gFieldState.closestBallInFront] :*/ gFieldState.balls[gFieldState.closestBall];
		const ObjectPosition &gate = gFieldState.gates[gRobotState.targetGate];
		double gateHeading = gate.heading;
		double ballHeading = ball.heading;
		double ballDistance = ball.distance;
		double rotation = 0;
		double errorMargin = 5;
		double maxDistance = 40;
		if (ballDistance > 10000) return DRIVEMODE_2V2_DEFENSIVE;
		if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		double heading = 0;
		double speed = 0;
		if (ballDistance > maxDistance) {
			heading = ballHeading;// +sign(gateHeading) / ballDistance;
			if (fabs(heading) > 30) heading = sign0(heading)*(fabs(heading) + 15);
			speed = std::max(60.0, ballDistance);
		}
		else {
			if (fabs(ballHeading) <= errorMargin && fabs(gateHeading) <= errorMargin){
				return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
			}
			if (fabs(ballHeading) > errorMargin){
				heading = ballHeading + sign0(ballHeading) * 55;
				speed = 60;
			}
			rotation = 0;
			if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		}
		m_pCom->Drive(speed, heading, rotation);
		return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
	}
};


class DriveToBallAimPartner : public DriveInstruction
{
public:
	DriveToBallAimPartner(const std::string &name = "2V2_DRIVE_TO_BALL_AIM_PARTNER") : DriveInstruction(name) {};
		DriveMode step(double dt)
		{
			ObjectPosition partnerPosition = gFieldState.gates[gRobotState.homeGate];
			partnerPosition.heading -= 20;
			auto &target = gFieldState.ballsFront[gFieldState.closestBallTribbler];

			if (target.distance > 10000) {
				/*
					Robot must stay in center ring. If lost ball, then start rotating.
				*/
				//m_pCom->Drive(0, 0, 15);
				return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER;
			}

			if (preciseAim(target, partnerPosition, speed, 10)) {
				if (m_pCom->BallInTribbler()){
					m_pCom->Kick(800);
					return DRIVEMODE_2V2_DEFENSIVE;
				}
			}
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation/2);
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER;
		}
};


std::pair<DriveMode, DriveInstruction*> MasterDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new MasterModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER, new DriveToBallAimPartner()),
	
};

std::pair<DriveMode, DriveInstruction*> SlaveDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new SlaveModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_CATCH_KICKOFF, new CatchKickOff()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate2v2()),
};

MultiModePlay::MultiModePlay(ISoccerRobot *pComModule, bool bMaster) :StateMachine(pComModule,
	bMaster ? TDriveModes(MasterDriveModes, MasterDriveModes + sizeof(MasterDriveModes) / sizeof(MasterDriveModes[0]))
	: TDriveModes(SlaveDriveModes, SlaveDriveModes + sizeof(SlaveDriveModes) / sizeof(SlaveDriveModes[0])))
	, isMaster(bMaster){}


MultiModePlay::~MultiModePlay(){}
