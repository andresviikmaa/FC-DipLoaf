// ColorCalibrator.cpp : Defines the entry point for the console application.
//
#ifdef WIN_32
#include "stdafx.h"
#endif
#include <opencv2/highgui.hpp>
#include <boost/program_options.hpp>

#include "../DisplayModule/Dialog.h"
#include "../HardwareModule/Camera.h"
#include "AutoCalibrator.h"
#include "../CommonModule/Settings.h"

namespace po = boost::program_options;
po::options_description desc("Allowed options");


int main(int argc, char* argv[])
{
	//cv::Mat green = cv::imread("field.png", 1);
	//cv::imshow("green", green);
	//cv::waitKey(0);
	//return 0;
	desc.add_options()
		("help", "produce help message")
		("name", po::value<std::string>(), "set Camera config file name");


	po::variables_map config;

	po::store(po::parse_command_line(argc, argv, desc), config);
	po::notify(config);

	if (config.count("help") || !config.count("name")) {
		std::cout << desc << std::endl;
		cv::waitKey(0);
		return -1;
	}

	std::atomic_bool exit;
	exit = false;
	Settings settings;
	std::string name = config["name"].as<std::string>();
	std::cout << "Initializing Camera... " << std::endl;
	Camera cam(name, name == "main"? settings.mainCam :settings.frontCam);
	std::cout << "Done" << std::endl;


	//Dialog display("Color Calibrator", cam.GetFrameSize(), cam.GetFrameSize());

	AutoCalibrator calibrator(&cam);
	//calibrator.Run();
	while (calibrator.running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
}

