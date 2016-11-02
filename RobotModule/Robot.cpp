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
#include "../VisionModule/FrontCameraVision.h"
#include "../CommonModule/FieldState.h"

extern FieldState gFieldState;

#define STATE_BUTTON(name, shortcut, new_state) \
m_pDisplay->createButton(std::string("") + name, shortcut, [&](){ this->SetState(new_state); });
#define BUTTON(name, shortcut, function_body) \
m_pDisplay->createButton(name, shortcut, [&]() function_body);
#define START_DIALOG if (state != last_state) { \
m_pDisplay->clearButtons();
#define END_DIALOG } \
last_state = (STATE)state; 




//TODO: convert to commandline options
//#define USE_ROBOTIINA_WIFI
#ifdef USE_ROBOTIINA_WIFI 
// robotiina wifi
boost::asio::ip::address bind_addr = boost::asio::ip::address::from_string("10.0.0.6"); // this computer ip
boost::asio::ip::address brdc_addr = boost::asio::ip::address::from_string("10.0.0.15"); // netmask 255.255.255.240
#else
// any local network
boost::asio::ip::address bind_addr = boost::asio::ip::address::from_string("0.0.0.0"); // all interfaces
boost::asio::ip::address brdc_addr = boost::asio::ip::address_v4::broadcast(); // local network
#endif

Robot::Robot(ICamera *pCamera, ISerial* pSerial, IDisplay*pDisplay)
{
	m_pVision = new FrontCameraVision(pCamera, pDisplay);
	m_pDisplay = pDisplay;
	m_pComModule = new ComModule(pSerial);

	assert(OBJECT_LABELS.size() == NUMBER_OF_OBJECTS);
	autoPilotEnabled = false;

}
Robot::~Robot()
{
}


bool Robot::Launch(const std::string &play_mode)
{
	/* Logic modules */
	if (play_mode == "master" || play_mode == "slave") {
		m_pAutoPilot = new MultiModePlay(m_pComModule, play_mode == "master");
	}
	else {
		m_pAutoPilot = new SingleModePlay(m_pComModule);
	}

	Run();

	return true;
}

void Robot::Run()
{
	double t1 = (double)cv::getTickCount();


	std::stringstream subtitles;
	try {
		while (true)
		{
			double t2 = (double)cv::getTickCount();
			double dt = (t2 - t1) / cv::getTickFrequency();
			t1 = t2;

			m_pVision->ProcessFrame(dt);
			m_pComModule->ProcessCommands();
			m_pAutoPilot->Step(dt);

			subtitles.str("");
			//subtitles << oss.str();
			subtitles << "|" << m_pAutoPilot->GetDebugInfo();
			subtitles << "|" << m_pComModule->GetDebugInfo();
			//if (!m_pComModule->IsReal()) {
			//	subtitles << "|" << "WARNING: Serial not connected!";
			//} 

			m_pDisplay->putShadowedText("fps: " + std::to_string(m_pVision->GetCamera()->GetFPS()), cv::Point(-140, 20), 0.5, cv::Scalar(255, 255, 255));
			//assert(STATE_END_OF_GAME != state);
			m_pDisplay->putShadowedText(std::string("running: ") + (gFieldState.isPlaying ? "yes" : "no"), cv::Point(-140, 460), 0.5, cv::Scalar(255, 255, 255));
			//m_pDisplay->putShadowedText( std::string("Ball:") + (ballPos.distance > 0 ? "yes" : "no"), cv::Point(-140, 60), 0.5, cv::Scalar(255, 255, 255));
			//m_pDisplay->putShadowedText( std::string("Gate:") + (targetGatePos.distance >0 ? "yes" : "no"), cv::Point(-140, 80), 0.5, cv::Scalar(255, 255, 255));


			m_pDisplay->putShadowedText(std::string("Trib:") + (m_pComModule->BallInTribbler() ? "yes" : "no"), cv::Point(-140, 100), 0.5, cv::Scalar(255, 255, 255));
			m_pDisplay->putShadowedText(std::string("Trib-x:") + (m_pComModule->BallInTribbler(true) ? "yes" : "no"), cv::Point(-140, 80), 0.5, cv::Scalar(255, 255, 255));
			m_pDisplay->putShadowedText(std::string("Sight:") + (gFieldState.gateObstructed ? "obst" : "free"), cv::Point(-140, 120), 0.5, cv::Scalar(255, 255, 255));
			//m_pDisplay->putShadowedText( std::string("OnWay:") + (somethingOnWay ? "yes" : "no"), cv::Point(-140, 140), 0.5, cv::Scalar(255, 255, 255));

			//const BallPosition &ball = gFieldState.balls.getClosest();
			//m_pDisplay->putShadowedText(std::string("Ball") + ": " + std::to_string(ball.polarMetricCoords.x) + " : " + std::to_string(ball.getHeading()), cv::Point(-250, 140), 0.4, cv::Scalar(255, 255, 255));

			m_pDisplay->putShadowedText(std::string("Collison border") + ": " + (gFieldState.collisionWithBorder ? "yes" : "no"), cv::Point(-250, 160), 0.4, cv::Scalar(255, 255, 255));
			m_pDisplay->putShadowedText(std::string("Collison unknown") + ": " + (gFieldState.collisionWithUnknown ? "yes" : "no"), cv::Point(-250, 180), 0.4, cv::Scalar(255, 255, 255));

			m_pDisplay->putShadowedText(std::string("Collison range") + ": " + std::to_string(gFieldState.collisionRange.x) + "- " + std::to_string(gFieldState.collisionRange.y), cv::Point(-250, 200), 0.4, cv::Scalar(255, 255, 255));

			/*
			const BallPosition &ballp = gFieldState.balls.getClosest(true);
			m_pDisplay->putShadowedText(std::string("Ball'")+ ": " + std::to_string(ballp.polarMetricCoords.x) + " : " + std::to_string(ballp.getHeading()), cv::Point(-250, 160), 0.4, cv::Scalar(255, 255, 255));
			*/
			/*
			for (int i = 0; i < gFieldState.balls.size(); i++) {

				BallPosition &ball = gFieldState.balls[i];
				m_pDisplay->putShadowedText( std::string("Ball") + std::to_string(i) + ": "+ std::to_string(ball.polarMetricCoords.x) + " : " + std::to_string(ball.polarMetricCoords.y), cv::Point(-250, i * 15 + 10), 0.3, cv::Scalar(255, 255, 255));
			}
			*/
			//std::stringstream ss;
			//ss.precision(3);
			//ss << "robot x:" << gFieldState.self.gFieldStateCoords.x<< " y: "<<gFieldState.self.gFieldStateCoords.y<< " r: " << gFieldState.self.angle;
			//m_pDisplay->putShadowedText(ss.str(), cv::Point(-250, 240), 0.4, cv::Scalar(255, 255, 255));
			//Simulator *pSim = dynamic_cast<Simulator*>(m_pVision);
			//if (pSim != NULL){
			//	ss.str("");
			//	ss << "sim   x:" << pSim->self.gFieldStateCoords.x << " y: " << pSim->self.gFieldStateCoords.y << " r: " << pSim->self.angle;
			//	m_pDisplay->putShadowedText(ss.str(), cv::Point(-250, 260), 0.4, cv::Scalar(255, 255, 255));
			//}


			//m_pDisplay->putShadowedText( "border: " + std::to_string(borderDistance.distance), cv::Point(-140, 280), 0.5, cv::Scalar(255, 255, 255));

			m_pDisplay->putShadowedText("Blue gate d: " + std::to_string((int)gFieldState.blueGate.distance) + " a: " + std::to_string(gFieldState.blueGate.angle - 180 * sign0(gFieldState.blueGate.angle)), cv::Point(-250, 280), 0.4, cv::Scalar(255, 255, 255));
			//		if (pSim != NULL)
			//			m_pDisplay->putShadowedText("Blue gate d: " + std::to_string((int)pSim->blueGate.distance) + " a: " + std::to_string(pSim->blueGate.angle), cv::Point(-250, 280), 0.4, cv::Scalar(255, 255, 255));
			m_pDisplay->putShadowedText("Yell gate d: " + std::to_string((int)gFieldState.yellowGate.distance) + " a: " + std::to_string(gFieldState.yellowGate.angle - 180 * sign0(gFieldState.yellowGate.angle)), cv::Point(-250, 310), 0.4, cv::Scalar(255, 255, 255));
			//		if (pSim != NULL)
			//			m_pDisplay->putShadowedText("Yell gate d: " + std::to_string((int)pSim->yellowGate.distance) + " a: " + std::to_string(pSim->yellowGate.angle), cv::Point(-250, 330), 0.4, cv::Scalar(255, 255, 255));

			m_pDisplay->putShadowedText("Partner d: " + std::to_string((int)gFieldState.partner.distance) + " a: " + std::to_string(gFieldState.partner.angle), cv::Point(-250, 360), 0.4, cv::Scalar(255, 255, 255));

			//TODO: fix putShadowedText newline thing
			std::vector<std::string> subtitles2;
			std::string subtitles3 = subtitles.str();

			boost::split(subtitles2, subtitles3, boost::is_any_of("|"));

			int j = 0;
			for (auto s : subtitles2) {
				if (s.empty()) s = " "; // opencv 3 crashes on empty string
				m_pDisplay->putShadowedText(s, cv::Point(10, -150 + j), 0.5, cv::Scalar(255, 255, 255));
				j += 20;
			}

			/*
			int key = cv::waitKey(1);
			if (key != -1) {
				KeyPressed(key);
			}
			if (key == 27) {
				std::cout << "exiting program" << std::endl;
				break;
			}
			*/
			//		frames++;

		}
	}
	catch (...) {
		std::cout << "Terminating robot" << std::endl;
		;
	}


	if (m_pAutoPilot != NULL)
		delete m_pAutoPilot;
	if (m_pComModule != NULL)
		delete m_pComModule;
	if (m_pVision != NULL)
		delete m_pVision;
//	refCom->setField(NULL);
}

