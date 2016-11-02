#include "MultiModePlay.h"
#include "SingleModePlay.h"
//#include "DistanceCalculator.h"
//#include "AutoPlayHelpers.h"
#include "../CommonModule/Types.h"
#include "../CommonModule/FieldState.h"

const float KICKOFF_ANGLE = 45.;
extern FieldState gFieldState;

enum MultiModeDriveStates {
	//2v2 modes
	DRIVEMODE_2V2_OFFENSIVE = 100,
	DRIVEMODE_2V2_DEFENSIVE,
	DRIVEMODE_2V2_WAIT_KICKOFF,
	DRIVEMODE_2V2_CATCH_KICKOFF,
	DRIVEMODE_2V2_AIM_PARTNER,
	//DRIVEMODE_AIM_GATE,
	//DRIVEMODE_2V2_KICK,
	//DRIVEMODE_DRIVE_TO_BALL,
	//DRIVEMODE_CATCH_BALL,
	DRIVEMODE_2V2_DRIVE_HOME,
	DRIVEMODE_2V2_OPPONENT_KICKOFF,
	DRIVEMODE_2V2_GOAL_KEEPER,
	DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE,
	DRIVEMODE_2V2_CATCH_BALL_NAIVE,
	DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE,


};

class DriveToBallNaivev2 : public DriveToBall
{
public:
	int colisionTicker = 0;
	Speed lastSpeed;
	DriveToBallNaivev2(const std::string &name = "DRIVE_TO_BALL_NAIVE") : DriveToBall(name){};

	DriveMode step(double dt)
	{
		speed = {0,0, 0};
		auto &target = gFieldState.closestBall;
		if (target.distance < 50) {
			m_pCom->ToggleTribbler(true);
		} else if (target.distance > 170) {
			m_pCom->ToggleTribbler(false);
		}
		
		if (m_pCom->BallInTribbler(true)) return DRIVEMODE_CATCH_BALL;
		if (aimTarget(target, speed, 10)){
			if (driveToTarget(target, speed, 35)) {
				if (aimTarget(target, speed, 1)) {
					return DRIVEMODE_2V2_CATCH_BALL_NAIVE;
				}
			}
		}
		std::cout << "1 " << target.heading << " " << speed.velocity << " " << speed.heading << " " << speed.rotation << std::endl;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE;
	}
};

class CatchBallNaivev2 : public DriveToBall
{
public:
	int colisionTicker = 0;
	Speed lastSpeed;
	CatchBallNaivev2(const std::string &name = "CATCH_BALL_NAIVE") : DriveToBall(name){};
	void onEnter(){ 
		m_pCom->ToggleTribbler(true);
	}
	DriveMode step(double dt)
	{
		if (STUCK_IN_STATE(3000)) return DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE;

		lastSpeed.velocity += 100*dt; 

		m_pCom->Drive(lastSpeed.velocity, lastSpeed.heading, lastSpeed.rotation);
		return DRIVEMODE_2V2_CATCH_BALL_NAIVE;
	}
};

class DriveToBallv2 : public DriveToBall
{
public:
	DriveToBallv2(const std::string &name = "DRIVE_TO_BALL_V2") : DriveToBall(name){};
	void onEnter(){ 
		m_pCom->ToggleTribbler(false);
	}

	DriveMode step(double dt)
	{

		auto &target = gFieldState.closestBall;

		if (m_pCom->BallInTribbler(true)) {
			std::cout << "BallInTribbler" << std::endl;
			return DRIVEMODE_AIM_GATE;
		}
		if (driveToTargetWithAngle(target, speed, 25, 5)){return DRIVEMODE_CATCH_BALL;}
		else {
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_DRIVE_TO_BALL;
		}

	}
};

class CatchBall2v2 : public CatchBall
{
private:
	boost::posix_time::ptime catchStart;
	bool master;
public:
	CatchBall2v2(bool mode) : CatchBall("2V2_CATCH_BALL"), master(mode){};
	virtual DriveMode step(double dt){
			if (m_pCom->BallInTribbler(true)){
				std::cout << "BallInTribbler" << std::endl;
				if (gFieldState.gameMode == GAME_MODE_START_OUR_KICK_OFF)return DRIVEMODE_2V2_AIM_PARTNER;
				else return DRIVEMODE_2V2_OFFENSIVE;
			};

	const BallPosition & target = gFieldState.closestBall;
	double heading = target.heading;
		if (/*STUCK_IN_STATE(3000) ||*/ target.distance > (initDist + 10)) return DRIVEMODE_DRIVE_TO_BALL;
	speed.velocity, speed.heading, speed.rotation = 0;
	if (fabs(target.heading) <= 2.) {
		if (catchTarget(target, speed)) {
			//if (gFieldState.gameMode == GAME_MODE_START_OUR_KICK_OFF)return DRIVEMODE_2V2_AIM_PARTNER;
			//else return DRIVEMODE_2V2_OFFENSIVE;
			
		}
		speed.rotation = - sign0(heading) * std::min(40.0, std::max(fabs(heading),5.0));

	}
	else {
		double heading = sign0(target.heading)*10.;
		//move slightly in order not to get stuck
		speed.velocity = 50;
		speed.rotation = -heading;
	}
	m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
	return DRIVEMODE_CATCH_BALL;

/*
	if(m_pCom->BallInTribbler()){
		if (gFieldState.gameMode == GAME_MODE_START_OUR_KICK_OFF)return DRIVEMODE_2V2_AIM_PARTNER;
		else return DRIVEMODE_2V2_OFFENSIVE;

	}
	FIND_TARGET_BALL //TODO: use it?
	if (STUCK_IN_STATE(3000) || target.distance > initDist  + 10) return DRIVEMODE_DRIVE_TO_BALL;
	speed.velocity, speed.heading, speed.rotation = 0;
	if(fabs(target.heading) <= 2) { 
		if(catchTarget(target, speed)){
			if (gFieldState.gameMode == GAME_MODE_START_OUR_KICK_OFF)return DRIVEMODE_2V2_AIM_PARTNER;
				else return DRIVEMODE_2V2_OFFENSIVE;

		}
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_CATCH_BALL;
	}
	double heading = sign(target.heading)*10;
	//move slightly in order not to get stuck
	if(heading == 0) m_pCom->Drive(-10,0, 0);
	else m_pCom->Drive(0,0, heading);
	return DRIVEMODE_DRIVE_TO_BALL;
	*/
	}
	
};

class MasterModeIdle : public Idle {

	virtual DriveMode step(double dt) {
		switch (gFieldState.gameMode) {
		case GAME_MODE_START_OPPONENT_KICK_OFF:
			return DRIVEMODE_2V2_OPPONENT_KICKOFF;
		case GAME_MODE_START_OPPONENT_THROWIN:
			return DRIVEMODE_2V2_OPPONENT_KICKOFF;
		case GAME_MODE_START_OPPONENT_FREE_KICK:
			return DRIVEMODE_2V2_OPPONENT_KICKOFF;
		case GAME_MODE_START_OUR_KICK_OFF:
		case GAME_MODE_START_OUR_FREE_KICK:
		case GAME_MODE_START_OUR_THROWIN:
			return gFieldState.isPlaying ? DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE : DRIVEMODE_IDLE;
		}
		return DRIVEMODE_IDLE;
	}
};
class SlaveModeIdle : public Idle {

	virtual DriveMode step(double dt) {
		//while (!gFieldState.isPlaying) return DRIVEMODE_IDLE;
		switch (gFieldState.gameMode) {
		case GAME_MODE_START_OPPONENT_KICK_OFF:
			return DRIVEMODE_2V2_DEFENSIVE;
		case GAME_MODE_START_OPPONENT_THROWIN:
			return DRIVEMODE_2V2_DEFENSIVE;
		case GAME_MODE_START_OPPONENT_FREE_KICK:
			return DRIVEMODE_2V2_DEFENSIVE;
		case GAME_MODE_START_OUR_KICK_OFF:
		case GAME_MODE_START_OUR_FREE_KICK:
			return DRIVEMODE_2V2_CATCH_KICKOFF;
		case GAME_MODE_START_OUR_THROWIN:
			return DRIVEMODE_2V2_DEFENSIVE;
		}
		return DRIVEMODE_IDLE;
	}
};
class Offensive : public DriveInstruction
{
public:
	Offensive() : DriveInstruction("2V2_OFFENSIVE"){};
	virtual DriveMode step(double dt){
		if (m_pCom->BallInTribbler(true)){
			//Reverese to GOAL			
			const ObjectPosition &gate = gFieldState.targetGate;
			double reverseHeading = gate.heading - 180 * sign0(gate.heading);
			double targetHeading = gate.heading;
			double targetDistance = gate.distance;

			double rotation = 0;
			double errorMargin = 5;
			double maxDistance = 80;
			if (fabs(reverseHeading) > errorMargin) rotation = -sign0(reverseHeading) * std::min(30.0, std::max(fabs(reverseHeading), 5.0));
			double heading = 0;
			double speed = 0;
			if (targetDistance > maxDistance) {
				heading = targetHeading;
				if (fabs(heading) > 30) heading = sign0(heading)*(fabs(heading) + 15);
				speed = 60;//std::max(60.0, targetDistance);
			}
			else {
				m_pCom->Drive(0,0,0);
				std::cout<<"to aim gate " << targetDistance << " " << maxDistance<<std::endl;
				return DRIVEMODE_AIM_GATE;
			}
			m_pCom->Drive(speed, heading, rotation);
			return DRIVEMODE_2V2_OFFENSIVE;
		}
		else return DRIVEMODE_CATCH_BALL;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
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
			//auto & opponent = gFieldState.targetGate; for testing
			auto & homeGate = gFieldState.homeGate;
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

class CatchKickOff : public DriveToBallv2
{
private:
	bool active = false;
public:
	CatchKickOff() : DriveToBallv2("2V2_CATCH_KICKOFF"){};

	virtual DriveMode step(double dt){
		if (gFieldState.gameMode == GAME_MODE_TAKE_BALL) {
			gFieldState.gameMode = GAME_MODE_IN_PROGRESS;
			return DRIVEMODE_DRIVE_TO_BALL;
		}
		
		return DRIVEMODE_2V2_CATCH_KICKOFF;
	}
};

class AimPartner : public DriveInstruction
{
protected:
	double initialHeading = 0;
public:
	AimPartner() : DriveInstruction("2V2_AIM_PARTNER"){};
	void onEnter(){ 
		initialHeading = gFieldState.homeGate.polarMetricCoords.y;
	}
	virtual DriveMode step(double dt){

		//auto & target = gFieldState.partner;
		auto target = gFieldState.homeGate;
		//std::cout << target.polarMetricCoords.y << std::endl;
		if (aimTarget(target, speed, KICKOFF_ANGLE)){
			m_pCom->Drive(0, 0, sign0(gFieldState.self.heading)*20);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			m_pCom->Kick(2500);
			assert(false);//TODO: fix this -> gFieldState.SendPartnerMessage("PAS #");
			std::this_thread::sleep_for(std::chrono::milliseconds(1500));
			//return DRIVEMODE_2V2_DRIVE_HOME;
			//return DRIVEMODE_IDLE;
			return DRIVEMODE_2V2_DEFENSIVE;
		}
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_2V2_AIM_PARTNER;
	}
};


class AimGate2v2 : public DriveInstruction
{
public:
	AimGate2v2() : DriveInstruction("2V2_AIM_GATE"){};
	virtual DriveMode step(double dt){

		ObjectPosition &lastGatePosition = gFieldState.targetGate;
		bool sightObstructed = gFieldState.gateObstructed;
		if (!m_pCom->BallInTribbler(true)) return DRIVEMODE_2V2_OFFENSIVE;
		if (aimTarget(lastGatePosition, speed, 2)){
			if (sightObstructed) { //then move sideways away from gate
				speed.velocity = 45;
				speed.heading += 90;
			}
			else return DRIVEMODE_KICK;
		}
		else {
			speed.rotation = lastGatePosition.heading;
			if(fabs(speed.rotation) > 50) speed.rotation = sign0(speed.rotation)*50;				
		}
		m_pCom->Drive(speed.velocity, speed.heading, -speed.rotation);
		return DRIVEMODE_AIM_GATE;
	}
};



class Kick2v2 : public DriveInstruction
{
public:
	Kick2v2() : DriveInstruction("2V2_KICK"){};
	virtual void onEnter();
	virtual DriveMode step(double dt);
};



class DriveHome2v2 : public DriveInstruction
{
public:
	DriveHome2v2() : DriveInstruction("2V2_DRIVE_HOME"){};
	virtual DriveMode step(double dt){
		ObjectPosition &lastGatePosition = gFieldState.homeGate;
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
		ball = gFieldState.closestBall;
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); //half second wait.
	}

	DriveMode step(double dt){
		if (ball.distance > 500) return DRIVEMODE_2V2_DEFENSIVE;
		const BallPosition& newBall = gFieldState.closestBall;
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
		auto &target = gFieldState.closestBall;
		if (target.distance == 0.0) {
			speed = { 0, 0, 0 };
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_2V2_GOAL_KEEPER;
		}
		if (target.distance < 35 && !gFieldState.obstacleNearBall) {
			return DRIVEMODE_DRIVE_TO_BALL;
		}
		auto homeGate = gFieldState.homeGate;
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
		if (m_pCom->BallInTribbler()) return DRIVEMODE_2V2_AIM_PARTNER;

		const ObjectPosition &ball = gFieldState.closestBall;
		const ObjectPosition &gate = gFieldState.homeGate;
		double gateHeading = gate.heading;
		double ballHeading = ball.heading;
		double ballDistance = ball.distance;
		double rotation = 0;
		double errorMargin = 5;
		double maxDistance = 40;
		if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		double heading = 0;
		double speed = 0;
		if (ballDistance > maxDistance) {
			heading = ballHeading;// +sign(gateHeading) / ballDistance;
			if (fabs(heading) > 30) heading = sign0(heading)*(fabs(heading) + 15);
			speed = std::max(60.0, ballDistance);
		}
		else {
			if (fabs(ballHeading) <= errorMargin && fabs(gateHeading) <= errorMargin) {
				m_pCom->Drive(0, 0, 0);
				return DRIVEMODE_CATCH_BALL;
			}
			if (fabs(ballHeading) > errorMargin) {
				heading = ballHeading + sign0(ballHeading) * 45;
				speed = 35;
			}
			rotation = 0;
			if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
			// drive around the ball
			//heading = ballHeading + sign(ballHeading) * 90;
			//std::max(fabs(ballHeading), 35.0);
		}
		m_pCom->Drive(speed, heading, rotation);
		return DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE;
	}
};

std::pair<DriveMode, DriveInstruction*> MasterDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new MasterModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL, new DriveToBallv2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_CATCH_BALL, new CatchBall2v2(true)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OFFENSIVE, new Offensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_AIM_PARTNER, new AimPartner()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_AIM_GATE, new AimGate2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_KICK, new Kick()),
//	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_KICKOFF, new KickOff()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OPPONENT_KICKOFF, new OpponentKickoff(true)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE, new DriveToBallNaivev2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate2v2())
};

std::pair<DriveMode, DriveInstruction*> SlaveDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new SlaveModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL, new DriveToBallv2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_CATCH_BALL, new CatchBall2v2(false)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DEFENSIVE, new Defensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OFFENSIVE, new Offensive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL, new DriveToBallv2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_HOME, new DriveHome2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_AIM_GATE, new AimGate2v2()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_CATCH_KICKOFF, new CatchKickOff()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_KICK, new Kick()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_OPPONENT_KICKOFF, new OpponentKickoff(false)),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_GOAL_KEEPER, new GoalKeeper()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_2V2_DRIVE_TO_BALL_NAIVE, new DriveToBallNaivev2())
//	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_CATCH_BALL, new CatchBall()),
};

MultiModePlay::MultiModePlay(ICommunicationModule *pComModule, bool bMaster) :StateMachine(pComModule,
	bMaster ? TDriveModes(MasterDriveModes, MasterDriveModes + sizeof(MasterDriveModes) / sizeof(MasterDriveModes[0]))
	: TDriveModes(SlaveDriveModes, SlaveDriveModes + sizeof(SlaveDriveModes) / sizeof(SlaveDriveModes[0])))
	, isMaster(bMaster){}


MultiModePlay::~MultiModePlay(){}

/*BEGIN Kick2v2*/
void Kick2v2::onEnter(){
	DriveInstruction::onEnter();
	m_pCom->ToggleTribbler(0);
}

DriveMode Kick2v2::step(double dt){
	m_pCom->ToggleTribbler(0);
	m_pCom->Drive(0, 0, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	m_pCom->Kick(2500);
	std::this_thread::sleep_for(std::chrono::milliseconds(500)); //half second wait.
	return DRIVEMODE_2V2_DEFENSIVE;
}
