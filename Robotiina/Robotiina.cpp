// Robotiina.cpp : Defines the entry point for the console application.
//
#ifdef WIN32
#include "stdafx.h"
#endif

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
#include "opencv2/highgui.hpp"
#ifdef SHOW_UI
#include "../DisplayModule/Dialog.h";
#endif

boost::asio::io_service io;
boost::asio::io_service io2;
std::atomic_bool exitRobot;

#ifdef WIN32
BOOL CtrlHandler(DWORD fdwCtrlType)
{ 
	switch( fdwCtrlType ) 
	{
		// Handle the CTRL-C signal. 
	case CTRL_C_EVENT:
		printf("Ctrl-C event\n\n");
		Beep(750, 300);
		exitRobot = true;
		return(TRUE);

		// CTRL-CLOSE: confirm that the user wants to exit. 
	case CTRL_CLOSE_EVENT:
		Beep(600, 200);
		printf("Ctrl-Close event\n\n");
		exitRobot = true;
		return(TRUE);

		// Pass other signals to the next handler. 
	case CTRL_BREAK_EVENT:
		Beep(900, 200);
		printf("Ctrl-Break event\n\n");
		return FALSE;

	case CTRL_LOGOFF_EVENT:
		Beep(1000, 200);
		printf("Ctrl-Logoff event\n\n");
		return FALSE;

	case CTRL_SHUTDOWN_EVENT:
		Beep(750, 500);
		printf("Ctrl-Shutdown event\n\n");
		return FALSE;

	default:
		return FALSE;
	}
}
#endif
#ifdef SHOW_UI
Dialog dialog("Robotiina", cv::Size(800, 600));
IDisplay * display(&dialog);
#endif

int main(int argc, char* argv[])
{
#ifdef WIN32
	//if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE)) {
	//  printf("\nERROR: Could not set control handler");
	//  return 1;
	//};
#endif
	try {
#ifdef SHOW_UI
		display->createButton("Exit", 'q', []{
			exitRobot = true;
		});
#endif
		Settings settings;
		settings.LoadFromCommandLine(argc, argv);

		std::cout << "Initializing Main Camera... " << std::endl;
		Camera mainCamera("main", settings.mainCam);
		std::cout << "Initializing Front Camera... " << std::endl;
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
	cv::waitKey();
	return 0;
}


