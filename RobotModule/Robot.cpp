#include "Robot.h"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/timer/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "../StateMachine/SingleModePlay.h"
#include "../StateMachine/MultiModePlay.h"
#include "../HardwareModule/ComModule.h"
#include "../VisionModule/MainCameraVision.h"
#include "../VisionModule/FrontCameraVision.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"
#include "ManualControl.h"
#include "../PredictionModule/RobotTracker.h"

FieldState gFieldState;
FieldState gPartnerFieldState;
RobotState gRobotState;
RobotState gPartnerRobotState;
extern std::atomic_bool exitRobot;
extern std::map<OBJECT, std::string> OBJECT_LABELS;

#define STATE_BUTTON(name, shortcut, new_state) \
m_pDisplay->createButton(std::string("") + name, shortcut, [&](){ this->SetState(new_state); });
#define BUTTON(name, shortcut, function_body) \
m_pDisplay->createButton(name, shortcut, [&]() function_body);
#define START_DIALOG if (state != last_state) { \
m_pDisplay->clearButtons();
#define END_DIALOG } \
last_state = (STATE)state; 


class StopAndDoNothing :
	public IStateMachine
{
protected:
	ISoccerRobot *m_pComModule;
public:
	StopAndDoNothing(ISoccerRobot *pComModule) :m_pComModule(pComModule) {};
	~StopAndDoNothing() {};
	virtual void Step(double dt) {
		m_pComModule->Drive(0, 0, 0);
		m_pComModule->ToggleTribbler(0);
	};
	virtual void enableTestMode(bool enable) {}
	virtual std::string GetDebugInfo() {
		return "Not Running";
	}
	virtual void Enable(bool enable) {};

};

enum COMMAND : uchar {
	COMMAND_FIELD_STATE = 0,
	COMMAND_ROBOT_STATE = 1,
	COMMAND_SET_PLAY_MODE = 10,
	COMMAND_SET_CONF = 11,
	COMMAND_MANUAL_CONTROL = 20,
	COMMAND_STATEMACHINE_STATE = 30,
	COMMAND_DEBUG = 100,
	COMMAND_DEBUG_STEP = 101
};



Robot::Robot(boost::asio::io_service &io, ICamera *pMainCamera, ICamera *pFrontCamera, ISoccerRobot* pSoccerRobot, bool master)
	: io(io), UdpServer(io, 30000, master), master(master)
{
	m_pMainVision = new MainCameraVision(pMainCamera);
	m_pFrontVision = new FrontCameraVision(pFrontCamera);
	m_pComModule =pSoccerRobot;

	assert(OBJECT_LABELS.size() == NUMBER_OF_OBJECTS);
	autoPilotEnabled = false;
	gFieldState.stateSize = sizeof(FieldState);
	gRobotState.stateSize = sizeof(RobotState);
}
Robot::~Robot()
{
}


bool Robot::Launch()
{
	m_AutoPilots.insert(std::make_pair(ROBOT_MODE_IDLE, new StopAndDoNothing(m_pComModule)));
	m_AutoPilots.insert(std::make_pair(ROBOT_MODE_1VS1, new SingleModePlay(m_pComModule)));
	m_AutoPilots.insert(std::make_pair(ROBOT_MODE_2VS2, new MultiModePlay(m_pComModule, master)));
//	m_AutoPilots.insert(std::make_pair("manual", new ManualControl(m_pComModule)));

	Run();

	return true;
}
void Robot::SendFieldState() {
	//const char * pData = reinterpret_cast<const char*>(&gFieldState);
	//SendData(pData, sizeof(FieldState));
	const char * pData2 = reinterpret_cast<const char*>(&gRobotState);
	SendData(pData2, sizeof(RobotState));

}
bool Robot::MessageReceived(const boost::array<char, BUF_SIZE>& buffer, size_t size) {
	COMMAND code = (COMMAND)buffer[0];
	if (code == COMMAND_FIELD_STATE && size == sizeof(FieldState)) {
		memcpy(&gPartnerFieldState, &buffer, size);
		return true;
	}
	else if (code == COMMAND_ROBOT_STATE && size == sizeof(RobotState)) {
		memcpy(&gPartnerRobotState, &buffer, size);
		return true;
	}
	else if (code == COMMAND_SET_PLAY_MODE) {
		RunMode newMode = (RunMode)buffer[1];
		if (m_AutoPilots.find(newMode) != m_AutoPilots.end()) {
			m_AutoPilots[gRobotState.runMode]->Enable(false);
			gRobotState.runMode = newMode;
			m_AutoPilots[newMode]->Enable(true);

		}
		return true;
	}
	else if (code == COMMAND_DEBUG) {
		m_pComModule->Drive(0, 0, 0);
		debug = !debug;
		return true;
	}
	else if (code == COMMAND_DEBUG_STEP) {		
		debug_step = true;
		return true;
	}
	else if (code == COMMAND_FIELD_STATE && size == sizeof(FieldState)) {
		// memcpy partner state
		return false;
	}
	return false;
};

bool Robot::MessageReceived(const std::string & message) {
	
	if (lastMessage != message){
		std::cout << "MessageReceived: " << message << std::endl;
		lastMessage = message;
	}
	
	if (message.empty()) return false;
	COMMAND code = (COMMAND)message[0];

	if (code == COMMAND_SET_CONF) {
		char val = message[1];
		char val2 = message[2];
		std::string key = message.substr(3);

		if (key == "field") gRobotState.FIELD_MARKER = val2;
		if (key == "team") gRobotState.TEAM_MARKER = val2;
		if (key == "marker") gRobotState.ROBOT_MARKER = val2;

		if (key == "gate") {
			gRobotState.targetGate = (OBJECT)(val + BLUE_GATE);
			gRobotState.homeGate = (OBJECT)(!val + BLUE_GATE);
		}
		if (key == "robot") {
			gRobotState.ourTeam = (OBJECT)(val + TEAM_PINK);
			gRobotState.oppoonentTeam = (OBJECT)(!val + TEAM_PINK);
		}
	}
	return true;
};
void Robot::InitializeTarget1vs1(){
	return;
	auto &B = gFieldState.gates[BLUE_GATE];
	auto &Y = gFieldState.gates[YELLOW_GATE];
	if (B.distance > Y.distance || Y.heading < 0) {
		gRobotState.targetGate = BLUE_GATE;
		gRobotState.homeGate = YELLOW_GATE;
	}
	else {
		gRobotState.targetGate = BLUE_GATE;
		gRobotState.homeGate = YELLOW_GATE;
	}
}
void Robot::Run()
{
	exitRobot = false;
	double t1 = (double)cv::getTickCount();
	double u1 = (double)cv::getTickCount();
#define GUSTAV
#ifdef GUSTAV
	if (false){//2v2
		gRobotState.runMode = ROBOT_MODE_2VS2;

		gRobotState.FIELD_MARKER = 'B';
		gRobotState.targetGate = YELLOW_GATE;
		gRobotState.homeGate = BLUE_GATE;
		//gRobotState.gameMode = GAME_MODE_START_OUR_PENALTY;
	}
	else{
		gRobotState.runMode = ROBOT_MODE_1VS1;
		gRobotState.FIELD_MARKER = 'B';
		gRobotState.targetGate = YELLOW_GATE;
		gRobotState.homeGate = BLUE_GATE;
		gRobotState.gameMode = GAME_MODE_START_PLAY;
	}

	//debug = true;

#endif
	std::stringstream subtitles;
	double fps = 0.;
	size_t counter = 0;
	//m_AutoPilots[curPlayMode]->Reset();
	lastGameMode = GAME_MODE_END_HALF;
	try {
		m_pMainVision->Enable(true);
		m_pFrontVision->Enable(true);
		RobotTracker robotTracker;
		bool frontUpdated = false;
		bool mainUpdated = false;
		while (!exitRobot)
		{
			if (lastGameMode != GAME_MODE_START_PLAY && gRobotState.gameMode == GAME_MODE_START_PLAY){
				InitializeTarget1vs1();
				//robotTracker.Reset();

			}
			double t2 = (double)cv::getTickCount();
			double dt = (t2 - t1) / cv::getTickFrequency();	
			double u2 = (double)cv::getTickCount();
			double du = (u2 - u1) / cv::getTickFrequency();


			mainUpdated = m_pMainVision->PublishState();
			frontUpdated = m_pFrontVision->PublishState();
			m_pComModule->ProcessCommands();
			if(gFieldState.closestBallTribbler > 3) robotTracker.Predict(dt, mainUpdated, frontUpdated);
			//TODO: remove this if ball in tribbler is working

			m_pComModule->SetBallInTribbler(gFieldState.ballsFront[gFieldState.closestBallTribbler].distance < 100 && fabs(gFieldState.ballsFront[gFieldState.closestBallTribbler].heading) < 15);
			
			if (du > 0.5) {
				fps = (double)counter / du;
				u1 = u2;
				counter = 0;
				SendFieldState();
			}
			counter++;
			io.poll();
			// MessageReceived handled 
			if (debug_step) {
				dt = 1;
			}
			if (!debug || debug_step) {
				m_AutoPilots[gRobotState.runMode]->Step(dt);
			}
			m_pComModule->SendMessages();
#ifdef SHOW_UI
			robotTracker.Draw();
#endif
			if (debug_step) {
				std::cout << "debug step: " << gFieldState.self.distance << " " << gFieldState.self.heading << " " << gFieldState.self.angle << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
				m_pComModule->Drive(0, 0, 0);
				debug_step = false;
			}
			subtitles.str("");
			//subtitles << oss.str();
			//subtitles << "|" << m_pAutoPilot->GetDebugInfo();
			//subtitles << "|" << 
			
			std::string debug = " " + m_AutoPilots[gRobotState.runMode]->GetDebugInfo();
			debug[0] = COMMAND_STATEMACHINE_STATE;
			SendData(debug.c_str(), debug.size());

//			std::string debug2 = " " + m_pComModule->GetDebugInfo();
//			debug2[0] = COMMAND_WHEELS_STATE;
//			SendData(debug2.c_str(), debug2.size());
			int ms = 20;
			std::chrono::milliseconds dura(ms);
			std::this_thread::sleep_for(dura);

			//int key = cv::waitKey(50);
			//if (key == 27) {
			//	std::cout << "exiting program" << std::endl;
			//	break;
			//}
			t1 = t2;

		}
	}
	catch (...) {
		std::cout << "Terminating robot" << std::endl;
		;
	}

	for(auto pilot: m_AutoPilots)
		delete pilot.second;

	if (m_pMainVision != NULL)
		delete m_pMainVision;
	if (m_pFrontVision != NULL)
		delete m_pFrontVision;
	//	refCom->setField(NULL);
}

