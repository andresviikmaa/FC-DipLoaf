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
		//return DRIVEMODE_DRIVE_HOME_AT_START;
	}
	if (gRobotState.gameMode != GAME_MODE_IN_PROGRESS) {
		return DRIVEMODE_IDLE;
	}
	return DRIVEMODE_DRIVE_TO_BALL; //DRIVEMODE_DRIVE_HOME_AT_START;
}

/*BEGIN DriveToBall*/
void DriveToBall::onEnter()
{
	toggledDribbler = false;
	DriveInstruction::onEnter();	
	
	initialBall = gFieldState.balls[gFieldState.closestBall];
	initialGate = gFieldState.gates[gRobotState.targetGate];
	m_pCom->ToggleTribbler(0);

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
		if (gFieldState.closestBallTribbler < 3) return DRIVEMODE_DRIVE_TO_BALL_FRONT;
		boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
		boost::posix_time::time_duration::tick_type collisionDt = (time - collisionTime).total_milliseconds();
		boost::posix_time::time_duration::tick_type collisionDt2 = (time - collisionTime2).total_milliseconds();

		if (collisionDt < 1000) speed = lastSpeed;
		else {
			auto &target = gFieldState.balls[gFieldState.closestBall];

			
			//auto &frontTarget = gFieldState.ballsFront[gFieldState.closestBallTribbler];

			if (target.distance > 10000) return DRIVEMODE_DRIVE_HOME;
			if (m_pCom->BallInTribbler()) return DRIVEMODE_CATCH_BALL;
			if (aimTarget(target, speed, 10))
				driveToTarget(target, speed, 35);
			//std::cout<<frontTarget.distance<<std::endl;
			//if (gFieldState.closestBallTribbler == 0)
			//	if (preciseAim(frontTarget, speed)) return DRIVEMODE_CATCH_BALL;

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
		std::cout << "output" << speed.velocity << speed.heading <<speed.rotation<< std::endl;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);		
		return DRIVEMODE_DRIVE_TO_BALL_NAIVE;
	}
};

class DriveToBallFront : public DriveToBall
{
public:

	DriveToBallFront(const std::string &name = "DRIVE_TO_BALL_FRONT") : DriveToBall(name){};


	DriveMode step(double dt)
	{
		//return DRIVEMODE_CATCH_BALL;
		if (m_pCom->BallInTribbler()) return DRIVEMODE_AIM_GATE;
		auto &target = gFieldState.ballsFront[gFieldState.closestBallTribbler];
		if (target.distance > 10000) return DRIVEMODE_DRIVE_TO_BALL;//target too far	
		if (preciseAim(target, speed, 2))  return DRIVEMODE_CATCH_BALL; 
		
		//std::cout << "output" << speed.velocity << speed.heading <<speed.rotation<< std::endl;
		m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
		return DRIVEMODE_DRIVE_TO_BALL_FRONT;
	}
};
class DriveToBallAngled : public DriveToBall
{
public:
	DriveToBallAngled(const std::string &name = "DRIVE_TO_BALL_ANGLED") : DriveToBall(name){};

	DriveMode step(double dt)
	{
		auto &target = gFieldState.balls[gFieldState.closestBall];

		bool useFront = false;
		if (useFront){
			auto &frontTarget = gFieldState.ballsFront[gFieldState.closestBallInFront];

			if (m_pCom->BallInTribbler()) return DRIVEMODE_CATCH_BALL;
			if (driveToTargetWithAngle(target, speed, 25, 5)){
				if (preciseAim(frontTarget, speed)) return DRIVEMODE_CATCH_BALL;
			}
			m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
			return DRIVEMODE_DRIVE_TO_BALL_ANGLED;
		}

		if (m_pCom->BallInTribbler()) return DRIVEMODE_CATCH_BALL;
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
		if (fabs(gFieldState.gates[gRobotState.targetGate].heading) < 20
			&& fabs(target.heading) > 120) {
			return DRIVEMODE_DRIVE_HOME;
		}
		if (target.distance > 10000) return DRIVEMODE_IDLE;
		if (m_pCom->BallInTribbler()) return DRIVEMODE_CATCH_BALL;
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
		if (m_pCom->BallInTribbler() || gFieldState.closestBallTribbler < 3) //if frontCam sees ball
			return DRIVEMODE_DRIVE_TO_BALL_FRONT;

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
		const ObjectPosition &ball = /*gFieldState.closestBallInFront != MAX_BALLS - 1 ? gFieldState.balls[gFieldState.closestBallInFront] :*/ gFieldState.balls[gFieldState.closestBall];
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
				speed = 50;
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
		if (gFieldState.closestBall < MAX_BALLS-1) return DRIVEMODE_DRIVE_TO_BALL;

		auto target = gFieldState.gates[gRobotState.homeGate];
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
		auto target = gFieldState.gates[gRobotState.homeGate];
		if (target.distance < 90) return DRIVEMODE_DRIVE_TO_BALL;
		//else m_pCom->Drive(90, 0, -sign0(target.heading)*20);
		else{
			const ObjectPosition &homeGate = gFieldState.gates[gRobotState.homeGate];
			const ObjectPosition &gate = gFieldState.gates[gRobotState.targetGate];
			double gateHeading = gate.heading;
			double ballHeading = -sign0(homeGate.heading)*(fabs(homeGate.heading)-35) ;
			double ballDistance = homeGate.distance;
			double rotation = 0;
			double errorMargin = 5;
			double maxDistance = 40;
			if (fabs(gateHeading) > errorMargin) rotation = sign0(gateHeading) * std::min(40.0, std::max(fabs(gateHeading), 5.0));
			double heading = 0;
			double speed = 0;
			if (ballDistance > maxDistance) {
				heading = ballHeading;// +sign(gateHeading) / ballDistance;
				if (fabs(heading) > 30) heading = -sign0(heading)*(fabs(heading) + 15);
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
	const BallPosition & target = gFieldState.ballsFront[gFieldState.closestBallTribbler];
	initDist = target.distance;
	m_pCom->Drive(0, 0, 0);
}
DriveMode CatchBall::step(double dt)
{
	if (m_pCom->BallInTribbler()){
		m_pCom->ToggleTribbler(250);
		return DRIVEMODE_AIM_GATE;
	}
	const BallPosition & target = gFieldState.ballsFront[gFieldState.closestBallTribbler];

	if (gFieldState.closestBallTribbler >3)  return DRIVEMODE_DRIVE_TO_BALL; //if frontCam doesn't see ball
	speed.velocity, speed.heading, speed.rotation = 0;
	if (fabs(target.heading) <= 2.) { //if heading correct 
		if (catchTarget(target, speed)) { //if ball in tribbler; sets velocity to 60
			m_pCom->ToggleTribbler(250);
			return DRIVEMODE_AIM_GATE;
		}
		speed.rotation = -(target.heading)*3;//random constant 
	}
	else return DRIVEMODE_DRIVE_TO_BALL_FRONT;
	m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);// velocity = 60; heading  = 0
	return DRIVEMODE_CATCH_BALL;
}

/*BEGIN AimGate*/

DriveMode AimGate::step(double dt)
{
	GatePosition target = gFieldState.gates[gRobotState.targetGate];
	target.heading -= 15; //compensate ball spin - to the left, to the left.
	if (!m_pCom->BallInTribbler()) return DRIVEMODE_DRIVE_TO_BALL; //lost ball, find new
	double errorMargin = (target.distance > 200) ? 1 : 2;
	if (aimTargetAroundBall(target, speed, errorMargin)) return DRIVEMODE_KICK;

	m_pCom->Drive(speed.velocity, speed.heading, speed.rotation);
	return DRIVEMODE_AIM_GATE;
}


/*BEGIN Kick*/
void Kick::onEnter()
{
	DriveInstruction::onEnter();
	m_pCom->ToggleTribbler(100);//slow down, don't stop, might loose ball
	m_pCom->Drive(0, 0, 0);
	//ACTIVE_DRIVE_TO_BALL_MODE = DRIVEMODE_DRIVE_TO_BALL_AIM_GATE;
}
DriveMode Kick::step(double dt)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	double dist = gFieldState.gates[gRobotState.targetGate].distance;
	m_pCom->Kick((dist > 500) ? 5000 : 2500); // if close use half kick strength
	std::this_thread::sleep_for(std::chrono::milliseconds(50)); //less than half second wait.
	m_pCom->ToggleTribbler(0);
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
		if (m_pCom->BallInTribbler())return DRIVEMODE_CATCH_BALL;
		const ObjectPosition &ball = gFieldState.balls[gFieldState.closestBall];
		const ObjectPosition &gate = gFieldState.gates[gRobotState.targetGate];
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
	std::pair<DriveMode, DriveInstruction*>(DRIVEMODE_DRIVE_TO_BALL_FRONT, new DriveToBallFront()),
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

SingleModePlay::SingleModePlay(ISoccerRobot *pComModule)
		:StateMachine(pComModule, TDriveModes(SingleDriveModes, SingleDriveModes + sizeof(SingleDriveModes) / sizeof(SingleDriveModes[0]))){};

