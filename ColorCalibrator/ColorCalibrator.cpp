// ColorCalibrator.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2/highgui.hpp>
#include <boost/program_options.hpp>

#include "../DisplayModule/dialog.h"
#include "../HardwareModule/Camera.h"
#include "AutoCalibrator.h"
#include "../CommonModule/Settings.h"

namespace po = boost::program_options;
po::options_description desc("Allowed options");


int main(int argc, char* argv[])
{
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


	Dialog display("Color Calibrator", cam.GetFrameSize(), cam.GetFrameSize());

	AutoCalibrator calibrator(&cam, &display);

	display.createButton("Reset", 'r', [&calibrator] {
		calibrator.Reset();
	});
	display.createButton("Take screenshot", 'c', [&calibrator] {
		calibrator.LoadFrame();
	});
	for (int i = 0; i < NUMBER_OF_OBJECTS; i++) {
		display.createButton(OBJECT_LABELS[(OBJECT)i], '-', [i, &calibrator] {
			calibrator.GetObjectThresholds(i, OBJECT_LABELS[(OBJECT)i]);
		});

	}
	display.createButton("Exit", 'x', [&exit]{
		exit = true;
	});
	while (!exit) {
		calibrator.Step();
		//key = cv::waitKey(30);
	}
}

