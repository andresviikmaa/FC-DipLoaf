// DistanceCalibrator.cpp : Defines the entry point for the console application.

#ifdef WIN_32
#include "stdafx.h"
#endif
#include <opencv2/highgui.hpp>
#include <boost/program_options.hpp>

#include "../DisplayModule/Dialog.h"
#include "../HardwareModule/Camera.h"
#include "DistanceCalibrator.h"
#include "../CommonModule/Settings.h"
#include <boost/property_tree/ini_parser.hpp>
#include "opencv2/calib3d.hpp"

DistanceCalibrator::DistanceCalibrator(ICamera * pCamera) :Dialog("Distance Calibrator", pCamera->GetFrameSize(), pCamera->GetFrameSize())
{
	m_pCamera = pCamera;
	frame_size = m_pCamera->GetFrameSize();
	counterValue = "NA";
	AddEventListener(this);
	points.push_back(std::make_tuple(cv::Point(-150, -225), cv::Point(0, 0), std::string("top left corner on cam image (blue gate is top)")));
	points.push_back(std::make_tuple(cv::Point(150, -225), cv::Point(0, 0), std::string("top right corner")));
	points.push_back(std::make_tuple(cv::Point(-156, 0), cv::Point(0, 0), std::string(" center left border")));
	points.push_back(std::make_tuple(cv::Point(-36, 0), cv::Point(0, 0), std::string(" center left circle")));
	points.push_back(std::make_tuple(cv::Point(36, 0), cv::Point(0, 0), std::string("center right circle")));
	points.push_back(std::make_tuple(cv::Point(150, 0), cv::Point(0, 0), std::string("center right border")));
	points.push_back(std::make_tuple(cv::Point(-150, 255), cv::Point(0, 0), std::string(" bottom left corner")));
	points.push_back(std::make_tuple(cv::Point(150, 255), cv::Point(0, 0), std::string("bottom right corner")));
	points.push_back(std::make_tuple(cv::Point(0, 0), cv::Point(0, 0), std::string("robot location on field image")));

	createButton("Start", 'x', [&]{
		Enable(true);
	});
	createButton("Exit", 'x', [&]{
		stop_thread = true;
	});
};

bool DistanceCalibrator::OnMouseEvent(int event, float x, float y, int flags) {
	if (enabled && event == cv::EVENT_LBUTTONUP) {
		std::get<1>(*itPoints) = cv::Point((int)(x), (int)(y));
		itPoints++;
		if (itPoints == points.end()){
			calculateDistances();
			message = "press backspace, calibration data saved to console";
			enabled = false;
		}
		else {
			message = std::get<2>(*itPoints);
		}
		return true;
	}
	return false;

};
struct less_than_key
{
	inline bool operator() (const cv::Point2d& struct1, const cv::Point2d& struct2)
	{
		return (struct1.x < struct2.x);
	}
};


void DistanceCalibrator::calculateDistances(){
	objectPoints.clear();
	imagePoints.clear();
	auto it = points.rbegin();
	for (it++; it != points.rend(); it++){
		objectPoints.push_back(cv::Vec3f(std::get<0>(*it).x, std::get<0>(*it).y));
		imagePoints.push_back(cv::Vec3f(std::get<1>(*it).x, std::get<1>(*it).y));

	}
	//cv::omnidir::calibrate()
}

double DistanceCalibrator::calculateDistance(double centreX, double centreY, double x, double y){
	return std::sqrt(std::abs(centreX - x)*std::abs(centreX - x) + std::abs(centreY - y)*std::abs(centreY - y));
}

void DistanceCalibrator::mouseClicked(int x, int y, int flags) {
	//std::cout << x << ", " << y << "--" << frame_size.x / 2 << ", " <<frame_size.y / 2 <<"  " << counter << "  dist: " << calculateDistance(frame_size.x / 2, frame_size.y / 2, x, y) << std::endl;
	counter = counter + DistanceCalibrator::DISTANCE_CALIBRATOR_STEP;
	std::ostringstream distance, value;
	distance << calculateDistance(frame_size.x / 2, frame_size.y / 2, x, y);
	value << counter;
	std::string distanceString = distance.str();
	std::string valueString = value.str();
	pt.put(valueString, distanceString);
	counterValue = valueString;
	std::cout << counter << std::endl;
	if (counter == VIEWING_DISTANCE){
		enabled = false;
		boost::property_tree::write_ini("distance_conf.ini", pt);
	}
	return;
}

void DistanceCalibrator::mouseClicked2(int x, int y, int flags) {
	std::cout << x << ", " << y
		<< ", d1: " << cv::norm(cv::Point(x, y) - cv::Point(304, 72))
		<< ", d2: " << cv::norm(cv::Point(x, y) - cv::Point(304, 534)) << std::endl;

}
void DistanceCalibrator::Enable(bool enable){
	enabled = enable;
	counterValue = "NA";
	counter = 0;
	pt.clear();
	itPoints = points.begin();
	message = enable ? std::get<2>(*itPoints) : "";

}

int DistanceCalibrator::Draw(){
	frameBGR = m_pCamera->Capture();
	putShadowedText(message, cv::Point(250, 220), 0.5, cv::Scalar(0, 0, 255));
	ShowImage(frameBGR);
	return Dialog::Draw();
}

DistanceCalibrator::~DistanceCalibrator(){
	RemoveEventListener(this);
}


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
	Camera cam(name, name == "main" ? settings.mainCam : settings.frontCam);
	std::cout << "Done" << std::endl;


	//Dialog display("Color Calibrator", cam.GetFrameSize(), cam.GetFrameSize());

	DistanceCalibrator calibrator(&cam);
	calibrator.Run();

}

