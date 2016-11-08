// Robotiina.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "../RobotModule/Robot.h"
#include <boost/algorithm/string.hpp>
#include "../HardwareModule/Camera.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "../HardwareModule/ComModule.h"
#include "../HardwareModule/SerialToUdp.h"
#include "../VisionModule/MainCameraVision.h"
#include "../StateMachine/SingleModePlay.h"
#include "../StateMachine/MultiModePlay.h"
#include "../CommonModule/Settings.h"

#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/exception/diagnostic_information.hpp> 
#include <boost/exception_ptr.hpp> 

namespace po = boost::program_options;

po::options_description desc("Allowed options");



boost::asio::io_service io;

int main(int argc, char* argv[])
{
	try {
		desc.add_options()
			("help", "produce help message")
			("main-camera", po::value<std::string>(), "set main camera index or path")
			("front-camera", po::value<std::string>(), "set front camera index or path")
			("app-size", po::value<std::string>(), "main window size: width x height")
			("locate_cursor", "find cursor instead of ball")
			("skip-ports", "skip ALL COM port checks")
			("skip-missing-ports", "skip missing COM ports")
			("save-frames", "Save captured frames to disc")
			("simulator-mode", po::value<std::string>(), "Play mode: single1, single2, opponent, master, slave")
			("play-mode", po::value<std::string>(), "Play mode: single, opponent, master, slave")
			("twitter-port", po::value<int>(), "UDP port for communication between robots")
			("mainboard-port", po::value<int>(), "mainboard UDP port")
			("mainboard-ip", po::value<std::string>(), "mainboard UDP ip address");


		std::string play_mode = "single";
		std::string simulator_mode = "single";

		po::variables_map config;

		po::store(po::parse_command_line(argc, argv, desc), config);
		po::notify(config);
		Settings settings;
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
		Camera * m_pFrontCamera;
		std::cout << "Initializing Main Camera... " << std::endl;
		if (config.count("main-camera"))
			m_pCamera = new Camera("main", config["main-camera"].as<std::string>());
		else
			m_pCamera = new Camera("main", "0");
		std::cout << "Done" << std::endl;

		std::cout << "Initializing Front Camera... " << std::endl;
		if (config.count("front-camera"))
			m_pFrontCamera = new Camera("front", config["front-camera"].as<std::string>());
		else
			m_pFrontCamera = new Camera("front", "0");

		std::cout << "Done" << std::endl;
		SerialToUdp mainboard(io, "127.0.0.2", 5000);
		Robot robot(io, m_pCamera, m_pFrontCamera, &mainboard, play_mode == "single1");
		robot.Launch(play_mode);

		if (m_pCamera) {
			delete m_pCamera;
		}
		if (m_pFrontCamera) {
			delete m_pFrontCamera;
		}
	}
	catch (const boost::exception const& e)
	{
		std::cout << boost::diagnostic_information(e) << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << "ups, " << e.what() << std::endl;
	}
	catch (const std::string &e)
	{
		std::cout << "ups, " << e << std::endl;
	}
	catch (...)
	{
		std::cout << "ups, did not see that coming." << std::endl;
	}
	return 0;
}


