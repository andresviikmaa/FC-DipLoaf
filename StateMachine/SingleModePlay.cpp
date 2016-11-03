#include "SingleModePlay.h"
#define ANDRESE_CATCH_BALL
#include "../CommonModule/Types.h"
#include "../CommonModule/FieldState.h"
extern FieldState gFieldState;

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
	if (!gFieldState.isPlaying) {
		return DRIVEMODE_IDLE;
	}
	return DRIVEMODE_DRIVE_HOME_AT_START;
}

/*BEGIN DriveToBall*/
void DriveToBall::onEnter()
{
	toggledDribbler = false;
	DriveInstruction::onEnter();	
	
	initialBall = gFieldState.balls[gFieldState.closestBall];
	initialGate = gFieldState.gates[gFieldState.targetGate];

	if (ACTIVE_DRIVE_TO_BALL_MODE == DRIVEMODE_IDLE)
		ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
}

DriveMode DriveToBall::step(double dt){
	if(initialBall.distance == 0) return DRIVEMODE_DRIVE_HOME;
	//return DRIVEMODE_DRIVE_TO_BALL_ANGLED;
	//return DRIVEMODE_DRIVE_TO_BALL_NAIVE;
	//return DRIVEMODE_ROTATE_AROUND_BALL;
	return ACTIVE_DRIVE_TO_BALL_MODE;
}
class DriveToBallNaive : public DriveToBall
{ 
public:
	int colisionTicker = 0;
	Speed lastSpeed;
	DriveToBallNaive(const std::string &name = "DRIVE_TO_BALL_NAIVE") : DriveToBall(name){};
	boost::posix_time::ptime collisionTime = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime collisionTime2 = boost::posix_time::microsec_clock::local_time();

	DriveMode step(double dt)
	{	
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration::tick_type collisionDt = (time - collisionTime).total_milliseconds();
		boost::posix_time::time_duration::tick_type collisionDt2 = (time - collisionTime2).total_milliseconds();

		if (collisionDt < 1000) speed = lastSpeed;
		else {
			auto &target = gFieldState.balls[gFieldState.closestBall];
			if (target.distance > 10000) return DRIVEMODE_IDLE;
			if (m_pCom->BallInTribbler()) return DRIVEMODE_AIM_GATE;
			if (aimTarget(target, speed, 10))
				if (driveToTarget(target, speed, 35))
					if (aimTarget(target, speed, 1))
						return DRIVEMODE_CATCH_BALL;
			if (gFieldState.collisionWithUnknown && fabs(speed.velocity) > 1.) {
				if (gFieldState.collisionRange.x < gFieldState.collisionRange.y) {
					if (speed.heading > gFieldState.collisionRange.x) {
						speed.heading -= 90;
						speed.velocity = 100;
					}
					else if (speed.heading < gFieldState.collisionRange.y){
						speed.heading += 90;
						speed.velocity = 100;
					}
				}
				speed.rotation = 0;
				collisionTime = boost::posix_time::microsec_clock::local_time();
				lastSpeed = speed;
			}
		}
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);		
		return DRIVEMODE_DRIVE_TO_BALL_NAIVE;
	}
};

class DriveToBallAngled : public DriveToBall
{
public:
	DriveToBallAngled(const std::string &name = "DRIVE_TO_BALL_ANGLED") : DriveToBall(name){};

	DriveMode step(double dt)
	{
		auto &target = gFieldState.balls[gFieldState.closestBall];

		if (m_pCom->BallInTribbler()) return DRIVEMODE_AIM_GATE;
		if (driveToTargetWithAngle(target, speed, 25, 5)){return DRIVEMODE_CATCH_BALL;}
		else {
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_DRIVE_TO_BALL_ANGLED;
		}
	}
};

class DriveToBallAvoidTurn : public DriveToBall
{
public:
	DriveToBallAvoidTurn(const std::string &name = "DRIVE_TO_BALL_AVOID_TURN") : DriveToBall(name){};

	DriveMode step(double dt)
	{
		
		auto &target = gFieldState.balls[gFieldState.closestBall];
		//if we are between closest ball and target gate and facing target gate then drive to home
		//to avoid rotating on any next ball
		if (fabs(gFieldState.gates[gFieldState.targetGate].heading) < 20
			&& fabs(target.heading) > 120) {
			return DRIVEMODE_DRIVE_HOME;
		}
		if (target.distance > 10000) return DRIVEMODE_IDLE;
		if (m_pCom->BallInTribbler()) return DRIVEMODE_AIM_GATE;
		//std::cout << std::endl << "aimtarget0, ";
		if (aimTarget(target, speed, 10))
			if (driveToTarget(target, speed))
				if (aimTarget(target, speed, 2)) return DRIVEMODE_CATCH_BALL;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_DIRVE_TO_BALL_AVOID_TURN;

	}
};
class DriveToBallAimGate : public DriveInstruction
{
public:
	DriveToBallAimGate(const std::string &name = "DRIVE_TO_BALL_AIM_GATE") : DriveInstruction(name){};

	DriveMode step(double dt){
		if (m_pCom->BallInTribbler())return DRIVEMODE_AIM_GATE;
		if (gFieldState.obstacleNearBall) {
			ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_ANGLED;
			return DRIVEMODE_DRIVE_TO_BALL_ANGLED;
		}
		/*
		if (gFieldState.collisionWithBorder) {
			ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_NAIVE;
			return DRIVEMODE_DRIVE_TO_BALL_NAIVE;
		}
		*/
		const ObjectPosition &ball = gFieldState.balls[gFieldState.closestBall];
		const ObjectPosition &gate = gFieldState.gates[gFieldState.targetGate];
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
			if (fabs(ballHeading) <= errorMargin && fabs(gateHeading) <= errorMargin){
				m_pCom->Drive(0, 0, 0);
				return DRIVEMODE_CATCH_BALL;
			}
			if (fabs(ballHeading) > errorMargin){
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
		return DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
	}
};
class DriveToHome : public DriveInstruction
{
public:
	DriveToHome(const std::string &name = "DRIVE_HOME") : DriveInstruction(name){};
	virtual DriveMode step(double dt){
		auto target = gFieldState.gates[gFieldState.homeGate];
		if (target.distance < 50) return DRIVEMODE_DRIVE_TO_BALL;
		else m_pCom->Drive(40, target.heading);
	return DRIVEMODE_DRIVE_HOME;
	}
};

class DriveHomeAtStart : public DriveInstruction
{
public:
	DriveHomeAtStart(const std::string &name = "DRIVE_HOME_AT_START") : DriveInstruction(name){};
	virtual DriveMode step(double dt){
		auto target = gFieldState.gates[gFieldState.homeGate];
		if (target.distance < 90) return DRIVEMODE_DRIVE_TO_BALL;
		//else m_pCom->Drive(90, 0, -sign0(target.heading)*20);
		else{
			const ObjectPosition &homeGate = gFieldState.gates[gFieldState.homeGate];
			const ObjectPosition &gate = gFieldState.gates[gFieldState.targetGate];
			double gateHeading = gate.heading;
			double ballHeading = sign0(homeGate.heading)*(fabs(homeGate.heading)-35) ;
			double ballDistance = homeGate.distance;
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
			m_pCom->Drive(speed, heading, 0);
		}
	return DRIVEMODE_DRIVE_HOME_AT_START;
	}
};



/*BEGIN CatchBall*/
void CatchBall::onEnter()
{
	DriveInstruction::onEnter();
	m_pCom->ToggleTribbler(250);
	const BallPosition & target = gFieldState.balls[gFieldState.closestBall];
	initDist = target.distance;
	m_pCom->Drive(0, 0, 0);
}
#ifdef ANDRESE_CATCH_BALL
DriveMode CatchBall::step(double dt)
{
	if (m_pCom->BallInTribbler())return DRIVEMODE_AIM_GATE;

	const BallPosition & target = gFieldState.balls[gFieldState.closestBall];
	double heading = target.heading;
		if (/*STUCK_IN_STATE(3000) ||*/ target.distance > (initDist + 10)) return DRIVEMODE_DRIVE_TO_BALL;
	speed.velocity, speed.heading, speed.rotation = 0;
	if (fabs(target.heading) <= 2.) {
		if (catchTarget(target, speed)) return DRIVEMODE_AIM_GATE;
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
}
#else
DriveMode CatchBall::step(double dt)
{
	FIND_TARGET_BALL //TODO: use it?
	if (STUCK_IN_STATE(3000) || target.distance > initDist  + 10) return DRIVEMODE_DRIVE_TO_BALL;
	speed.velocity, speed.heading, speed.rotation = 0;
	if(fabs(target.heading) <= 2) { 
		if(catchTarget(target, speed)) return DRIVEMODE_AIM_GATE;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_CATCH_BALL;
	}
	double heading = sign(target.heading)*10;
	//move slightly in order not to get stuck
	if(heading == 0) m_pCom->Drive(-10,0, 0);
	else m_pCom->Drive(0,0, heading);
	return DRIVEMODE_DRIVE_TO_BALL;
}
#endif
void CatchBall::onExit(){}//DO_NOT_STOP_TRIBBLER
/*END CatchBall*/


/*BEGIN AimGate*/

DriveMode AimGate::step(double dt)
{
	const GatePosition & target = gFieldState.gates[gFieldState.targetGate];
	if (!m_pCom->BallInTribbler()) return DRIVEMODE_DRIVE_TO_BALL;
	double errorMargin;
	if (target.distance > 200) errorMargin = 1;
	else errorMargin = 2;	
	if (aimTarget(target, speed, errorMargin)) return DRIVEMODE_KICK; 
	m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
	return DRIVEMODE_AIM_GATE;
}


/*BEGIN Kick*/
void Kick::onEnter()
{
	DriveInstruction::onEnter();
	m_pCom->ToggleTribbler(0);
	m_pCom->Drive(0, 0, 0);
	ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
}
DriveMode Kick::step(double dt)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	m_pCom->Kick(5000);
	std::this_thread::sleep_for(std::chrono::milliseconds(50)); //less than half second wait.
	return DRIVEMODE_DRIVE_TO_BALL;
}

class RotateAroundBall : public DriveInstruction
{
protected:
	double initialBallHeading = 0;
public:
	RotateAroundBall(const std::string &name = "ROTATE_AROUND_BALL") : DriveInstruction(name){};
	void onEnter(){ initialBallHeading = gFieldState.balls[gFieldState.closestBall].heading;}

	virtual DriveMode step(double dt){
		if (m_pCom->BallInTribbler())return DRIVEMODE_AIM_GATE;
		const ObjectPosition &ball = gFieldState.balls[gFieldState.closestBall];
		const ObjectPosition &gate = gFieldState.gates[gFieldState.targetGate];
		double gateHeading = gate.heading;
		double gateAngle = gate.angle;
		double ballAngle = ball.angle;
		double ballHeading = ball.heading;
		double ballDistance = ball.distance;
		double rotation = 0;
		double heading = 0;
		double speed = 0;
		double errorMargin = 5;
		double maxDistance = 30;
		if (fabs(gateHeading) <= errorMargin && fabs(ballHeading) <= errorMargin) return DRIVEMODE_CATCH_BALL;
		if (fabs(gateHeading) > errorMargin) rotation = -sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
		if (ballDistance > maxDistance) {
			maxDistance = 30;
			double top = 1;// (fabs(initialBallHeading) > 90) ? -1 : 1;
			heading = ballAngle + sign0(ballHeading) * top*asin(maxDistance / ballDistance) * 180 / PI;
			speed = std::max(60.0, ballDistance);
		}
		else if (fabs(gateHeading - ballHeading) > errorMargin/2){
			//return DRIVEMODE_CATCH_BALL;
			// drive around the ball
			double top = 1;// (fabs(initialBallHeading) > 90) ? -1 : 1;
			double left = sign0(initialBallHeading);
			heading = ballAngle + top*left*90;
			speed = 40;
			maxDistance = 60;
		}
		if (((speed) < 0.01) && (fabs(heading) < 0.01) && (fabs(rotation) < 0.01)) return DRIVEMODE_CATCH_BALL;// nowhere to go, error margins are out-of-sync
		m_pCom->Drive(speed, heading, rotation);
		return DRIVEMODE_ROTATE_AROUND_BALL;
	}
};



std::pair<DriveMode, DriveInstruction*> SingleDriveModes[] = {
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_IDLE, new SingleModeIdle()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_HOME, new DriveToHome()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_HOME_AT_START, new DriveHomeAtStart()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL, new DriveToBall()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_NAIVE, new DriveToBallNaive()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DIRVE_TO_BALL_AVOID_TURN, new DriveToBallAvoidTurn()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_ANGLED, new DriveToBallAngled()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_AIM_GATE, new DriveToBallAimGate()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_AIM_GATE, new AimGate()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_KICK, new Kick()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_CATCH_BALL, new CatchBall()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_ROTATE_AROUND_BALL, new RotateAroundBall()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_CRASH, new Crash()),
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_BORDER_TO_CLOSE, new BorderToClose()),
};

SingleModePlay::SingleModePlay(ICommunicationModule *pComModule)
		:StateMachine(pComModule, TDriveModes(SingleDriveModes, SingleDriveModes + sizeof(SingleDriveModes) / sizeof(SingleDriveModes[0]))){};

