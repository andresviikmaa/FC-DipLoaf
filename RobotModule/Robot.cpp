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


std::pair<OBJECT, std::string> objects[] = {
	std::pair<OBJECT, std::string>(BALL, "Ball"),
	std::pair<OBJECT, std::string>(BLUE_GATE, "Blue Gate"),
	std::pair<OBJECT, std::string>(YELLOW_GATE, "Yellow Gate"),
	std::pair<OBJECT, std::string>(FIELD, "Field"),
    std::pair<OBJECT, std::string>(INNER_BORDER, "Inner Border"),
	std::pair<OBJECT, std::string>(OUTER_BORDER, "Outer Border"),
//	std::pair<OBJECT, std::string>(NUMBER_OF_OBJECTS, "") // this is intentionally left out

};

std::map<OBJECT, std::string> OBJECT_LABELS(objects, objects + sizeof(objects) / sizeof(objects[0]));

std::pair<STATE, std::string> states[] = {
	std::pair<STATE, std::string>(STATE_NONE, "None"),
	std::pair<STATE, std::string>(STATE_AUTOCALIBRATE, "Autocalibrate"),
	std::pair<STATE, std::string>(STATE_CALIBRATE, "Manual calibrate"),
	std::pair<STATE, std::string>(STATE_LAUNCH, "Launch"),
	std::pair<STATE, std::string>(STATE_SETTINGS, "Settings"),
//	std::pair<STATE, std::string>(STATE_LOCATE_GATE, "Locate Gate"),
	std::pair<STATE, std::string>(STATE_REMOTE_CONTROL, "Remote Control"),
//	std::pair<STATE, std::string>(STATE_CRASH, "Crash"),
	std::pair<STATE, std::string>(STATE_RUN, "Autopilot"),
	std::pair<STATE, std::string>(STATE_TEST, "Test"),
	std::pair<STATE, std::string>(STATE_MANUAL_CONTROL, "Manual Control"),
	std::pair<STATE, std::string>(STATE_SELECT_GATE, "Select Gate"),
	std::pair<STATE, std::string>(STATE_DANCE, "Dance"),
	std::pair<STATE, std::string>(STATE_MOUSE_VISION, "Mouse Vision"),
	std::pair<STATE, std::string>(STATE_DISTANCE_CALIBRATE, "dist"),
	std::pair<STATE, std::string>(STATE_GIVE_COMMAND, "Give Referee Command"),
	//	std::pair<STATE, std::string>(STATE_END_OF_GAME, "End of Game") // this is intentionally left out

};

std::pair<std::string, GameMode> refCommands[] = {
//	std::pair<std::string, GameMode>("Start game", GAME_MODE_START_SINGLE_PLAY),
//	std::pair<std::string, GameMode>("Stop game", GAME_MODE_STOPED),
	std::pair<std::string, GameMode>("Placed ball", GAME_MODE_PLACED_BALL),
	std::pair<std::string, GameMode>("End half", GAME_MODE_END_HALF),

	std::pair<std::string, GameMode>("Our kickoff", GAME_MODE_START_OUR_KICK_OFF),
	std::pair<std::string, GameMode>("Our indirect free kick", GAME_MODE_START_OUR_INDIRECT_FREE_KICK),
	std::pair<std::string, GameMode>("Our direct free kick", GAME_MODE_START_OUR_FREE_KICK),
	std::pair<std::string, GameMode>("Our goal kick", GAME_MODE_START_OUR_GOAL_KICK),
	std::pair<std::string, GameMode>("Our throw in", GAME_MODE_START_OUR_THROWIN),
	std::pair<std::string, GameMode>("Our corner kick", GAME_MODE_START_OUR_CORNER_KICK),
	std::pair<std::string, GameMode>("Our penalty", GAME_MODE_START_OUR_PENALTY),
	std::pair<std::string, GameMode>("Our goal", GAME_MODE_START_OUR_GOAL),
	std::pair<std::string, GameMode>("Our yellow card", GAME_MODE_START_OUR_YELLOW_CARD),
/*
	std::pair<std::string, GameMode>("Opponent kickoff", GAME_MODE_START_OPPONENT_KICK_OFF),
	std::pair<std::string, GameMode>("Opponent indirect free kick", GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK),
	std::pair<std::string, GameMode>("Opponent direct free kick", GAME_MODE_START_OPPONENT_FREE_KICK),
	std::pair<std::string, GameMode>("Opponent goal kick", GAME_MODE_START_OPPONENT_GOAL_KICK),
	std::pair<std::string, GameMode>("Opponent throw in", GAME_MODE_START_OPPONENT_THROWIN),
	std::pair<std::string, GameMode>("Opponent corner kick", GAME_MODE_START_OPPONENT_CORNER_KICK),
	std::pair<std::string, GameMode>("Opponent penalty", GAME_MODE_START_OPPONENT_PENALTY),
	std::pair<std::string, GameMode>("Opponent goal", GAME_MODE_START_OPPONENT_GOAL),
	std::pair<std::string, GameMode>("Opponent yellow card", GAME_MODE_START_OPPONENT_YELLOW_CARD)
*/
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
boost::asio::ip::address brdc_addr = boost::asio::ip::address_v4::broadcast(); // local network
#endif

std::map<STATE, std::string> STATE_LABELS(states, states + sizeof(states) / sizeof(states[0]));

/* BEGIN DANCE MOVES */
void dance_step(float time, float &move1, float &move2) {
	move1 = 50*sin(time/4000);
	move2 = 360 * cos(time / 4000);
}

/* END DANCE MOVES */

Robot::Robot(ICamera *pCamera, ISerial* pSerial, IDisplay*pDisplay)
{
	m_pVision = new FrontCameraVision(pCamera, pDisplay);
	m_pDisplay = pDisplay;
	m_pComModule = new ComModule(pSerial);

	last_state = STATE_END_OF_GAME;
	state = STATE_NONE;
    //wheels = new WheelController(io);
	assert(OBJECT_LABELS.size() == NUMBER_OF_OBJECTS);
	assert(STATE_LABELS.size() == STATE_END_OF_GAME);
	autoPilotEnabled = false;

}
Robot::~Robot()
{
}


bool Robot::Launch(const std::string &play_mode)
{
	/*
	if (!ParseOptions(argc, argv)) return false;
	if (config.count("play-mode"))
		play_mode = config["play-mode"].as<std::string>();

	std::string _cam = config.count("camera") >0? config["camera"].as<std::string>():"";
	bool bSimulator = _cam == "simulator" || _cam == "simulator-master";
	if (bSimulator) {
		InitSimulator(_cam == "simulator-master", play_mode);
	}
	else {
		InitHardware();
	}
	std::cout << "Starting Robot" << std::endl;

	cv::Size winSize(0, 0);
	if (config.count("app-size")) {
		std::vector<std::string> tokens;
		boost::split(tokens, config["app-size"].as<std::string>(), boost::is_any_of("x"));
		winSize.width = atoi(tokens[0].c_str());
		winSize.height = atoi(tokens[1].c_str());

	}
	// Compose robot from its parts
	if (config.count("webui") == 0)
		m_pDisplay = new Dialog("Robotiina", winSize, m_pVision->GetFrameSize());
	else
		m_pDisplay = new WebUI(8080);
	captureFrames = config.count("capture-frames") > 0;
	*/

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
/*
void Robot::InitSimulator(bool master, const std::string game_mode) {
	auto pSim = new Simulator(io, master, game_mode);
	m_pVision = pSim;
	m_pSerial = pSim;
	refCom = pSim;
}

void Robot::InitHardware() {
	std::cout << "Initializing Camera... " << std::endl;
	if (config.count("camera"))
		if (config["camera"].as<std::string>() == "ximea")
			m_pVision = new Camera(CV_CAP_XIAPI);
		else
			m_pVision = new Camera(config["camera"].as<std::string>());
	else
		m_pVision = new Camera(0);
	std::cout << "Done" << std::endl;
	initRefCom();
	try {
		using boost::property_tree::ptree;
		ptree pt;
		read_ini("conf/ports.ini", pt);
		std::string port = pt.get<std::string>(std::to_string(ID_COM));
		m_pSerial = new SimpleSerial(io, port, 19200);
		//Sleep(100);
	}
	catch (std::exception const&  ex) {
		std::cout << "Main board com port fail: " << ex.what() << std::endl;
	}
	
	std::cout << "Done initializing" << std::endl;
	return;
}

void Robot::initRefCom() {
	std::cout << "Init referee communications" << std::endl;
	try {
		using boost::property_tree::ptree;
		ptree pt;
		read_ini("conf/ports.ini", pt);
		std::string port = pt.get<std::string>(std::to_string(ID_REF));
		refCom = new LLAPReceiver(NULL, io, port);
	}
	catch (...) { 
		std::cout << "Referee com LLAP reciever couldn't be initialized" << std::endl;
		refCom = new RefereeCom(NULL);
	}
}
*/
/*
void Robot::initCoilboard() {
	if (coilBoardPortsOk) {
		std::cout << "Using coilgun" << std::endl;
		try {
			using boost::property_tree::ptree;
			ptree pt;
			read_ini("conf/ports.ini", pt);
			std::string port = pt.get<std::string>(std::to_string(ID_COILGUN));
			coilBoard = new CoilBoard(io, port);
		}
		catch (...) {
			throw;
		}
	}
	else {
		std::cout << "WARNING: Not using coilgun" << std::endl;
		coilBoard = new CoilGun();
	}
}
*/
void Robot::RunCaptureTest()
{
	/*
	cv::Mat frameBGR = m_pVision->Capture();
	std::cout << frameBGR.type() << std::endl;
	while (true)
	{
		frameBGR = m_pVision->Capture();
		//cvtColor(frameBGR, frameBGR, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		//m_pDisplay->ShowImage(frameBGR);
		//m_pDisplay->putText("fps: " + std::to_string(m_pVision->GetFPS()), cv::Point(-140, 20), 0.5, cv::Scalar(255, 255, 255));
		cv::putText(frameBGR, std::to_string(m_pVision->GetFPS()), cv::Point(140, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0));
		cv::imshow("frame", frameBGR);
		cv::waitKey(1);
	}
	*/
}
void Robot::Run()
{
	//RunCaptureTest();
	//return;
	//double fps;
	//int frames = 0;
	//timer for rotation measure
	boost::posix_time::ptime lastStepTime;
	boost::posix_time::time_duration dt;

	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime epoch = boost::posix_time::microsec_clock::local_time();

	boost::posix_time::ptime rotateTime = time;
	boost::posix_time::time_duration rotateDuration;

	std::string captureDir;
	boost::posix_time::ptime captureStart = boost::posix_time::microsec_clock::local_time();
	cv::VideoWriter *outputVideo = NULL;

	/*= "videos/" + boost::posix_time::to_simple_string(time) + "/";
	std::replace(captureDir.begin(), captureDir.end(), ':', '.');
	if (captureFrames) {
	boost::filesystem::create_directories(captureDir);
	}
	*/

	/* Field state */

	//SoccerField gFieldState(io, m_pDisplay, play_mode == "master" || play_mode == "single", play_mode == "master" || play_mode == "slave" ? 1 : 11);
	//refCom->setField(&gFieldState);

	/* Vision modules */
	//AutoCalibrator m_pVision(m_pVision, this);
	//MouseVision mouseVision(m_pVision, m_pDisplay, &gFieldState);

	//AutoCalibrator calibrator(m_pVision, m_pDisplay);

	//DistanceCalibrator distanceCalibrator(m_pVision, m_pDisplay);

	/* Communication modules */


	//RobotTracker tracker(wheels);

	std::stringstream subtitles;

	while (true)
    {

		std::ostringstream oss;
		oss.precision(4);

		oss << "[Robot] State: " << STATE_LABELS[state];
		/*
		oss << ", Ball: " << (ballFound ? "yes" : "no");
		oss << ", Gate: " << (targetGatePos != NULL ? "yes" : "no");
		oss << ", trib: " << (ballInTribbler ? "yes" : "no");
		oss << ", Sight: " << (!sightObstructed ? "yes" : "no");
		oss << "|[Robot] Ball Pos: (" << ballPos.distance << "," << ballPos.horizontalAngle << "," << ballPos.horizontalDev << ")";
		if(targetGatePos != NULL)
			oss << "|[Robot] Gate Pos: (" << targetGatePos->distance << "," << targetGatePos->horizontalAngle << "," << targetGatePos->horizontalDev << ")";
		else
			oss << "|[Robot] Gate Pos: - ";
//		oss << "Gate Pos: (" << lastBallLocation.distance << "," << lastBallLocation.horizontalAngle << "," << lastBallLocation.horizontalDev << ")";
*/

		/* Main UI */
		if (STATE_NONE == state) {
			START_DIALOG
				m_pAutoPilot->enableTestMode(false);
				m_pComModule->Drive(0);
				STATE_BUTTON("(A)utoCalibrate objects", 'a', STATE_AUTOCALIBRATE)
				//STATE_BUTTON("(M)anualCalibrate objects", STATE_CALIBRATE)
				//STATE_BUTTON("(C)Change Gate [" + ((int)gFieldState.GetTargetGate().distance == (int)(gFieldState.blueGate.distance) ? "blue" : "yellow") + "]", 'c', STATE_SELECT_GATE)
				STATE_BUTTON("(G)ive Referee Command", 'g', STATE_GIVE_COMMAND)
				//STATE_BUTTON("Auto(P)ilot [" + (m_pAutoPilot->running ? "On" : "Off") + "]", 'p', STATE_LAUNCH)
				m_pDisplay->createButton(std::string("Referee : ") + (gFieldState.isPlaying ? "STOP" : "START"), 'v', [this]{
					gFieldState.isPlaying = !gFieldState.isPlaying;

					this->last_state = STATE_END_OF_GAME; // force dialog redraw
				});

				//m_pDisplay->createButton(std::string("Save video: ") + (m_pVision.captureFrames() ? "on" : "off"), 'v', [this, &m_pVision]{
				//	m_pVision.captureFrames(!m_pVision.captureFrames());
				//
				//	this->last_state = STATE_END_OF_GAME; // force dialog redraw
				//});
				//std::stringstream sset;
				//sset << " [ robot: " << refCom->FIELD_MARKER << refCom->ROBOT_MARKER << ", team: " << refCom->TEAM_MARKER << 
				//	", color: " << ((gFieldState.robotColor == ROBOT_COLOR_YELLOW_UP) ? "Yellow Up" : "Blue Up")  << "]";
				//
				//STATE_BUTTON("(S)ettings" + sset.str(), 's', STATE_SETTINGS)

				m_pDisplay->createButton("Swap displays", '-', [this] {
					m_pDisplay->SwapDisplays();
				});
				m_pDisplay->createButton("Toggle main display on/off", '-', [this] {
					m_pDisplay->ToggleDisplay();
				});
				//m_pDisplay->createButton("Pause/Play video", 'f', [this] {
				//	m_pVision->TogglePlay();
				//});




				STATE_BUTTON("Test Autopilot", '-', STATE_TEST)
				STATE_BUTTON("E(x)it", 27, STATE_END_OF_GAME)
			END_DIALOG
		}
		


		else if (STATE_SELECT_GATE == state) {
			START_DIALOG
				m_pDisplay->createButton(OBJECT_LABELS[BLUE_GATE], '-', [this]{
				//gFieldState.SetTargetGate(BLUE_GATE);
				//Simulator *pSim = dynamic_cast<Simulator*>(this->m_pVision);
				//if (pSim!=NULL){
				//	pSim->self.gFieldStateCoords = cv::Point2d(155, 230);
				//	pSim->self.polarMetricCoords.y = -30;
				//}
				this->SetState(STATE_NONE);
			});
			m_pDisplay->createButton(OBJECT_LABELS[YELLOW_GATE], '-', [this]{
				//gFieldState.SetTargetGate(YELLOW_GATE);
				//Simulator *pSim = dynamic_cast<Simulator*>(this->m_pVision);
				//if (pSim != NULL){
				//	pSim->self.gFieldStateCoords = cv::Point2d(-155, -230);
				//	pSim->self.polarMetricCoords.y = 150;
				//
				//}
				this->SetState(STATE_NONE);
			});
			END_DIALOG
		}
		else if (STATE_SETTINGS == state) {
			START_DIALOG
				IConfigurableModule *pModule = dynamic_cast<IConfigurableModule*>(m_pVision);
				for (auto setting : pModule->GetSettings()){
					m_pDisplay->createButton(setting.first + ": " + std::get<0>(setting.second)(), '-', [this, setting]{
						std::get<1>(setting.second)();
						this->last_state = STATE_END_OF_GAME; // force dialog redraw
					});
				}
				//pModule = static_cast<IConfigurableModule*>(refCom);
				//for (auto setting : pModule->GetSettings()){
				//	m_pDisplay->createButton(setting.first + ": " + std::get<0>(setting.second)(), '-', [this, setting]{
				//		std::get<1>(setting.second)();
				//		this->last_state = STATE_END_OF_GAME; // force dialog redraw
				//	});
				//}
				m_pDisplay->createButton(std::string("Our Color: ") + ((gFieldState.robotColor == ROBOT_COLOR_YELLOW_UP) ? "Yellow Up" : "Blue Up"), '-', [this]{
					gFieldState.robotColor = (RobotColor)(!gFieldState.robotColor);
					this->last_state = STATE_END_OF_GAME; // force dialog redraw
				});
				STATE_BUTTON("BACK", 8, STATE_NONE)
			END_DIALOG
		}
		else if (STATE_LAUNCH == state) {
			//if (targetGate == NUMBER_OF_OBJECTS) {
			//	std::cout << "Select target gate" << std::endl;
			//	SetState(STATE_SELECT_GATE);
			//}
			//else {
				try {
					/*
					CalibrationConfReader calibrator;
					for (int i = 0; i < NUMBER_OF_OBJECTS; i++) {
						objectThresholds[(OBJECT)i] = calibrator.GetObjectThresholds(i, OBJECT_LABELS[(OBJECT)i]);
					}
					*/
					//SetState(STATE_SELECT_GATE);
					m_pComModule->ToggleTribbler(0);
					m_pComModule->Drive(0);
					//m_pAutoPilot->Enable(!m_pAutoPilot->running);
					SetState(STATE_NONE);
				}
				catch (...){
					std::cout << "Calibration data is missing!" << std::endl;
					//TODO: display error
					SetState(STATE_AUTOCALIBRATE); // no conf
				}
			//}
		}
		//else if (STATE_TEST == state) {
		//	START_DIALOG
		//		m_pAutoPilot->enableTestMode(true);
		//		for (const auto d : m_pAutoPilot->driveModes) {
		//			m_pDisplay->createButton(d.second->name, '-', [this, d]{
		//				m_pAutoPilot->setTestMode(d.first);
		//			});
		//		}
		//		last_state = STATE_TEST;
		//		STATE_BUTTON("BACK", 8, STATE_NONE)
		//	END_DIALOG
		//}
		//else if (STATE_GIVE_COMMAND == state) {
		//	START_DIALOG
		//		for (std::pair < std::string, GameMode> entry : refCommands) {
		//			m_pDisplay->createButton(entry.first, '-', [this, entry]{
		//				refCom->giveCommand(entry.second);
		//			});
		//		}
		//	STATE_BUTTON("BACK", 8, STATE_NONE)
		//		END_DIALOG
		//}
		else if (STATE_END_OF_GAME == state) {
			break;
		}
		 
		subtitles.str("");
		//subtitles << oss.str();
		subtitles << "|" << m_pAutoPilot->GetDebugInfo();
		subtitles << "|" << m_pComModule->GetDebugInfo();
		//if (!m_pComModule->IsReal()) {
		//	subtitles << "|" << "WARNING: Serial not connected!";
		//} 

		m_pDisplay->putShadowedText( "fps: " + std::to_string(m_pVision->GetCamera()->GetFPS()), cv::Point(-140, 20), 0.5, cv::Scalar(255, 255, 255));
		//assert(STATE_END_OF_GAME != state);
		m_pDisplay->putShadowedText("state: " + STATE_LABELS[state], cv::Point(-140, 40), 0.5, cv::Scalar(255, 255, 255));
		m_pDisplay->putShadowedText(std::string("running: ") +(gFieldState.isPlaying ? "yes":"no"), cv::Point(-140, 460), 0.5, cv::Scalar(255, 255, 255));
		//m_pDisplay->putShadowedText( std::string("Ball:") + (ballPos.distance > 0 ? "yes" : "no"), cv::Point(-140, 60), 0.5, cv::Scalar(255, 255, 255));
		//m_pDisplay->putShadowedText( std::string("Gate:") + (targetGatePos.distance >0 ? "yes" : "no"), cv::Point(-140, 80), 0.5, cv::Scalar(255, 255, 255));

		
		m_pDisplay->putShadowedText( std::string("Trib:") + (m_pComModule->BallInTribbler() ? "yes" : "no"), cv::Point(-140, 100), 0.5, cv::Scalar(255, 255, 255));
		m_pDisplay->putShadowedText( std::string("Trib-x:") + (m_pComModule->BallInTribbler(true) ? "yes" : "no"), cv::Point(-140, 80), 0.5, cv::Scalar(255, 255, 255));
		m_pDisplay->putShadowedText( std::string("Sight:") + (gFieldState.gateObstructed ? "obst" : "free"), cv::Point(-140, 120), 0.5, cv::Scalar(255, 255, 255));
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
		m_pDisplay->putShadowedText("Yell gate d: " + std::to_string((int)gFieldState.yellowGate.distance) + " a: " + std::to_string(gFieldState.yellowGate.angle -180 * sign0(gFieldState.yellowGate.angle)), cv::Point(-250, 310), 0.4, cv::Scalar(255, 255, 255));
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
			m_pDisplay->putShadowedText( s, cv::Point(10, -150 + j), 0.5, cv::Scalar(255, 255, 255));
			j += 20;
		}

		/* robot tracker */
		//cv::Point2i center(-100, 200);
		//double velocity = 0, direction = 0, rotate = 0;
		//auto speed = m_pComModule->GetTargetSpeed();

		/*
		//Draw circle
		cv::Scalar colorCircle(133, 33, 55);
		cv::circle(display, center, 60, colorCircle, 2);
		*/
		//m_pDisplay->Draw();
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
    	
	if (outputVideo != NULL) {
		delete outputVideo;
	}

	if (m_pAutoPilot != NULL)
		delete m_pAutoPilot;
//	refCom->setField(NULL);
}

