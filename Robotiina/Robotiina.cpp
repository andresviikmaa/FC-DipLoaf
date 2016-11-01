// Robotiina.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "../RobotModule/Robot.h"
#include "../DisplayModule/Dialog.h"
#include <boost/algorithm/string.hpp>
#include "../HardwareModule/Camera.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "../HardwareModule/ComModule.h"
#include "../VisionModule/FrontCameraVision.h"
#include "../StateMachine/SingleModePlay.h"
#include "../StateMachine/MultiModePlay.h"

po::options_description desc("Allowed options");



boost::asio::io_service io;

int main(int argc, char* argv[])
{
	desc.add_options()
		("help", "produce help message")
		("camera", po::value<std::string>(), "set m_pCamera index or path")
		("app-size", po::value<std::string>(), "main window size: width x height")
		("locate_cursor", "find cursor instead of ball")
		("skip-ports", "skip ALL COM port checks")
		("skip-missing-ports", "skip missing COM ports")
		("save-frames", "Save captured frames to disc")
		("simulator-mode", po::value<std::string>(), "Play mode: single, opponent, master, slave")
		("play-mode", po::value<std::string>(), "Play mode: single, opponent, master, slave")
		("twitter-port", po::value<int>(), "UDP port for communication between robots");

	std::string play_mode = "single";
	std::string simulator_mode = "single";

	po::variables_map config;

	po::store(po::parse_command_line(argc, argv, desc), config);
	po::notify(config);

	if (config.count("help")) {
		std::cout << desc << std::endl;
		return false;
	}
	if (config.count("play-mode"))
		play_mode = config["play-mode"].as<std::string>();
	if (config.count("simulator-mode"))
		simulator_mode = config["simulator-mode"].as<std::string>();

	cv::Size winSize(1024, 768);
	if (config.count("app-size")) {
		std::vector<std::string> tokens;
		boost::split(tokens, config["app-size"].as<std::string>(), boost::is_any_of("x"));
		winSize.width = atoi(tokens[0].c_str());
		winSize.height = atoi(tokens[1].c_str());

	}
	Camera * m_pCamera;
	std::cout << "Initializing Camera... " << std::endl;
	if (config.count("camera"))
		if (config["camera"].as<std::string>() == "ximea")
			m_pCamera = new Camera(CV_CAP_XIAPI);
		else
			m_pCamera = new Camera(config["camera"].as<std::string>());
	else
		m_pCamera = new Camera(0);
	std::cout << "Done" << std::endl;
	Dialog display("Robotiina", winSize, m_pCamera->GetFrameSize());
	
	Robot robot(m_pCamera, NULL, &display);
	robot.Launch(play_mode);

	if (m_pCamera) {
		delete m_pCamera;
	}

	return 0;
}


