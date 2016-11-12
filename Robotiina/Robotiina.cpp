// Robotiina.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "../RobotModule/Robot.h"
#include <boost/algorithm/string.hpp>
#include "../HardwareModule/Camera.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "../HardwareModule/ComModule.h"
#include "../VisionModule/MainCameraVision.h"
#include "../StateMachine/SingleModePlay.h"
#include "../StateMachine/MultiModePlay.h"
#include "../CommonModule/Settings.h"


#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/exception/diagnostic_information.hpp> 
#include <boost/exception_ptr.hpp> 



boost::asio::io_service io;
boost::asio::io_service io2;

int main(int argc, char* argv[])
{
	try {

		Settings settings;
		settings.LoadFromCommandLine(argc, argv);

		std::cout << "Initializing Cameras... " << std::endl;
		Camera mainCamera("main", settings.frontCam);
		Camera frontCamera("front", settings.frontCam);
		std::cout << "Done" << std::endl;

		ComModule mainboard(io, settings.ethernetIp, settings.ethernetPort);
		std::cout << "Done" << std::endl;
		Robot robot(io, &mainCamera, &frontCamera, &mainboard, settings.master);
		robot.Launch();

	}
	catch (const boost::exception & e)
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


