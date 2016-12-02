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
	DRIVEMODE_2V2_OFFENSIVE = 100,
	DRIVEMODE_2V2_DEFENSIVE,
	DRIVEMODE_2V2_WAIT_KICKOFF,
	DRIVEMODE_2V2_CATCH_KICKOFF,
	DRIVEMODE_2V2_AIM_PARTNER,
	DRIVEMODE_2V2_DRIVE_HOME,
	DRIVEMODE_2V2_OPPONENT_KICKOFF,
	DRIVEMODE_2V2_GOAL_KEEPER,
	DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE,
	DRIVEMODE_2V2_CATCH_BALL_NAIVE,
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

class Offensive : public DriveInstruction
{
public:
	Offensive() : DriveInstruction("2V2_OFFENSIVE"){};
	virtual DriveMode step(double dt) {
		//TODO: regular 1v1 logic here
		return DRIVEMODE_2V2_OFFENSIVE;
	}
};

class Defensive : public DriveInstruction
{
public:
	Defensive() : DriveInstruction("2V2_DEFENSIVE"){};
	virtual DriveMode step(double dt){
		auto & target = gFieldState.partner;
		if (/*partner not goal keeper become goal keeper*/ true){ return DRIVEMODE_2V2_DRIVE_HOME;}//ToDo goalKeeper message 
		else{
			auto & opponent = gFieldState.opponents[0];//get the one with ball?
			//auto & opponent = gFieldState.gates[gRobotState.targetGate]; for testing
			auto & homeGate = gFieldState.gates[gRobotState.homeGate];
			double gateHeading = homeGate.heading - 180 * sign0(homeGate.heading);
			double gateAngle = homeGate.heading - 180 * sign0(homeGate.heading);;
			double opponentAngle = opponent.angle;
			double opponentHeading = opponent.heading;
			double opponentDistance = opponent.distance;
			double rotation = 0;
			double heading = 0;
			double velocity = 0;
			double errorMargin = 5;
			double maxDistance = 30;
			if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));

			if (opponentDistance > maxDistance) {
				maxDistance = 30;
				double top = 1;
				heading = opponentAngle + sign0(opponentHeading) * top*asin(maxDistance / opponentDistance) * 180 / PI;
				velocity = std::max(60.0, opponentDistance);
			}
			else if (fabs(gateHeading - opponentHeading) > errorMargin / 2){
				double top = 1;
				double left = sign0(opponentHeading);
				heading = opponentAngle + top*left * 90;
				velocity = 40;
				maxDistance = 60;
			}
			speed.velocity = velocity;
			speed.heading = heading;
			speed.rotation = rotation;
			
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
		if (gPartnerRobotState.driveState != DRIVEMODE_2V2_AIM_PARTNER) {
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

class OpponentKickoff : public DriveInstruction
{
public:
	OpponentKickoff(bool mode) : DriveInstruction("2V2_OPPONENT_KICKOFF"), mode(mode){};
	void onEnter(){
		ball = gFieldState.balls[gFieldState.closestBall];
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); //half second wait.
	}

	DriveMode step(double dt){
		if (ball.distance > 500) return DRIVEMODE_2V2_DEFENSIVE;
		const BallPosition& newBall = gFieldState.balls[gFieldState.closestBall];
		if (abs(ball.distance - newBall.distance) > 10 || abs(ball.angle - newBall.angle) > 5){ return DRIVEMODE_2V2_OFFENSIVE;	}
		else{
			std::this_thread::sleep_for(std::chrono::milliseconds(500)); //half second wait.
			return DRIVEMODE_2V2_OPPONENT_KICKOFF;
		}
	}
private:
	ObjectPosition ball;
	bool mode;
};

class GoalKeeper : public DriveInstruction
{
private:
	bool wentLeft = false;
public:
	GoalKeeper() : DriveInstruction("2V2_GOAL_KEEPER"){};

	virtual DriveMode step(double dt){
		auto &target = gFieldState.balls[gFieldState.closestBall];
		if (target.distance == 0.0) {
			speed = { 0, 0, 0 };
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_2V2_GOAL_KEEPER;
		}
		if (target.distance < 35 && !gFieldState.obstacleNearBall) {
			return DRIVEMODE_DRIVE_TO_BALL;
		}
		auto homeGate = gFieldState.gates[gRobotState.homeGate];
		double homeGateDist = homeGate.distance;
		double gateAngle = homeGate.heading - 180 * sign0(homeGate.heading);
		aimTarget(target, speed,2);	
		if (homeGateDist < 30 || homeGate.minCornerPolarCoords.x < 30) {
			driveToTargetWithAngle(target, speed, 40, 5);
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
		//TODO: aim partner and
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
			partnerPosition.heading -= 30;
			auto &target = gFieldState.ballsFront[gFieldState.closestBallTribbler];

			if (target.distance > 10000) {
				/*
					Robot must stay in center ring. If lost ball, then start rotating.
				*/
				m_pCom->Drive(0, 0, 30);
				return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER;
			}

			if (preciseAim(target, partnerPosition, speed, 2)) {
				if (m_pCom->BallInTribbler()){
					m_pCom->Kick(1000);
					return DRIVEMODE_2V2_DEFENSIVE;
				}
			}

			//std::cout << "output" << speed.velocity << speed.heading <<speed.rotation<< std::endl;
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_PARTNER;
		}
};


std::pair<DriveMode, DriveInstruction*> MasterDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new MasterModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OFFENSIVE, new Offensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_KICK, new Kick()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OPPONENT_KICKOFF, new OpponentKickoff(true)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate2v2())
};

std::pair<DriveMode, DriveInstruction*> SlaveDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new SlaveModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OFFENSIVE, new Offensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_CATCH_KICKOFF, new CatchKickOff()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_KICK, new Kick()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OPPONENT_KICKOFF, new OpponentKickoff(false)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
};

MultiModePlay::MultiModePlay(ISoccerRobot *pComModule, bool bMaster) :StateMachine(pComModule,
	bMaster ? TDriveModes(MasterDriveModes, MasterDriveModes + sizeof(MasterDriveModes) / sizeof(MasterDriveModes[0]))
	: TDriveModes(SlaveDriveModes, SlaveDriveModes + sizeof(SlaveDriveModes) / sizeof(SlaveDriveModes[0])))
	, isMaster(bMaster){}


MultiModePlay::~MultiModePlay(){}
