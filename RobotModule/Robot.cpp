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
#include "ManualControl.h"

extern FieldState gFieldState;

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
	ICommunicationModule *m_pComModule;
public:
	StopAndDoNothing(ICommunicationModule *pComModule) :m_pComModule(pComModule) {};
	~StopAndDoNothing() {};
	virtual void Step(double dt) {
		m_pComModule->Drive(0, 0, 0);
		m_pComModule->ToggleTribbler(0);
	};
	virtual void enableTestMode(bool enable) {}
	virtual std::string GetDebugInfo() {
		return "";
	}
	virtual void Enable(bool enable) {};

};



//TODO: convert to commandline options
//#define USE_ROBOTIINA_WIFI
#ifdef USE_ROBOTIINA_WIFI 
// robotiina wifi
boost::asio::ip::address bind_addr = boost::asio::ip::address::from_string("10.0.0.6"); // this computer ip
boost::asio::ip::address brdc_addr = boost::asio::ip::address::from_string("10.0.0.15"); // netmask 255.255.255.240
#else
// any local network
boost::asio::ip::address bind_addr = boost::asio::ip::address::from_string("0.0.0.0"); // all interfaces
//boost::asio::ip::address brdc_addr = boost::asio::ip::address_v4::broadcast(); // local network
boost::asio::ip::address brdc_addr = boost::asio::ip::address::from_string("127.255.255.255"); // netmask 255.255.255.240

#endif

Robot::Robot(boost::asio::io_service &io, ICamera *pMainCamera, ICamera *pFrontCamera, ISerial* pSerial, bool master)
	: io(io), UdpServer(io, 30000, master)
{
	m_pMainVision = new MainCameraVision(pMainCamera);
	m_pFrontVision = new FrontCameraVision(pFrontCamera);
	m_pComModule = new ComModule(pSerial);

	assert(OBJECT_LABELS.size() == NUMBER_OF_OBJECTS);
	autoPilotEnabled = false;
	gFieldState.stateSize = sizeof(FieldState);

}
Robot::~Robot()
{
}


bool Robot::Launch(const std::string &play_mode)
{
	m_AutoPilots.insert(std::make_pair("idle", new StopAndDoNothing(m_pComModule)));
	m_AutoPilots.insert(std::make_pair("1vs1", new SingleModePlay(m_pComModule)));
	m_AutoPilots.insert(std::make_pair("2vs2", new MultiModePlay(m_pComModule, play_mode == "master")));
	m_AutoPilots.insert(std::make_pair("manual", new ManualControl(m_pComModule)));

	Run();

	return true;
}
void Robot::SendFieldState() {

	const char * pData = reinterpret_cast<const char*>(&gFieldState);
	SendData(pData, sizeof(FieldState));

}

void Robot::MessageReceived(const std::string & message) {
	std::cout << "MessageReceived: " << message << std::endl;
};
void Robot::Run()
{
	double t1 = (double)cv::getTickCount();


	std::stringstream subtitles;
	double fps = 0.;
	size_t counter = 0;
	//m_AutoPilots[curPlayMode]->Reset();
	try {
		m_pMainVision->Enable(true);
		m_pFrontVision->Enable(true);
		while (true)
		{
			double t2 = (double)cv::getTickCount();
			double dt = (t2 - t1) / cv::getTickFrequency();
			if (counter > 10) {
				fps = (double)counter / dt;
				t1 = t2;
				counter = 0;
			}
			counter++;

			m_pMainVision->PublishState();
			m_pFrontVision->PublishState();
			m_pComModule->ProcessCommands();
			m_AutoPilots[curPlayMode]->Step(dt);

			//io.reset();
			//io.poll_one();

			SendFieldState();
			io.poll();
			// MessageReceived handled 
			/*
			subtitles.str("");
			//subtitles << oss.str();
			subtitles << "|" << m_pAutoPilot->GetDebugInfo();
			subtitles << "|" << m_pComModule->GetDebugInfo();
			*/
			int key = cv::waitKey(1);
			if (key == 27) {
				std::cout << "exiting program" << std::endl;
				break;
			}


		}
	}
	catch (...) {
		std::cout << "Terminating robot" << std::endl;
		;
	}

	for(auto pilot: m_AutoPilots)
		delete pilot.second;

	if (m_pComModule != NULL)
		delete m_pComModule;
	if (m_pMainVision != NULL)
		delete m_pMainVision;
	if (m_pFrontVision != NULL)
		delete m_pFrontVision;
	//	refCom->setField(NULL);
}

