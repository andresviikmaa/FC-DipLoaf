// ColorCalibrator.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/highgui.hpp"
#include <boost/program_options.hpp>

#include "../DisplayModule/dialog.h"
#include "../HardwareModule/Camera.h"
#include "AutoCalibrator.h"

namespace po = boost::program_options;
po::options_description desc("Allowed options");


int main(int argc, char* argv[])
{
	desc.add_options()
		("help", "produce help message")
		("camera", po::value<std::string>(), "set Camera index or path")
		("name", po::value<std::string>(), "set Camera config file name");


	po::variables_map config;

	po::store(po::parse_command_line(argc, argv, desc), config);
	po::notify(config);

	if (config.count("help")|| !config.count("camera") || !config.count("name")) {
		std::cout << desc << std::endl;
		cv::waitKey(0);
		return -1;
	}


	Camera cam(config["name"].as<std::string>(), config["camera"].as<std::string>());

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

	int key = 0;
	while (key != 27) {
		calibrator.Step();
		key = cv::waitKey(30);
	}
}

