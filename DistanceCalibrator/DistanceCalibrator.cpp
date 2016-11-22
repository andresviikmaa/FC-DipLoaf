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

DistanceCalibrator::DistanceCalibrator(ICamera * pCamera) :Dialog("Color Calibrator", pCamera->GetFrameSize(), pCamera->GetFrameSize())
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
};

bool DistanceCalibrator::OnMouseEvent(int event, float x, float y, int flags, bool bMainArea) {
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
		if (bMainArea)
			mouseClicked((int)(x), (int)(y), flags);
		else if (!bMainArea)
			mouseClicked2((int)(x), (int)(y), flags);
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

	pt.clear();
	auto it = points.rbegin();
	auto robotPosOnField = std::get<1>(*it);
	std::vector<cv::Point2d> conf;
	for (it++; it != points.rend(); it++){
		double dist1 = cv::norm(std::get<1>(*it) - frame_size / 2); // on cam from center
		double dist2 = cv::norm(std::get<0>(*it) - (robotPosOnField - cv::Point(303, 303))); // on field
		std::cout << dist2 << "=" << dist1 << std::endl;
		conf.push_back(cv::Point2d(dist2, dist1));
	}
	std::sort(conf.begin(), conf.end(), less_than_key());
	for (auto dists : conf){
		pt.put(std::to_string((int)round(dists.x)), std::to_string(dists.y));
	}
	boost::property_tree::write_ini("distance_conf.ini", pt);
	//gDistanceCalculator.loadConf();
	/*
	cv::Point2d blue1 = points[0].second;
	cv::Point2d yellow1 = points[1].second;
	cv::Point2d self1 = cv::Point2d(frame_size) / 2;
	cv::Point blue2(0, -250);
	cv::Point yellow2(0, 250);
	cv::Point self2 = points[2].second - cv::Point(304,304);

	double distanceToBlue1 = cv::norm(blue1 - self1);
	double distanceToYellow1 = cv::norm(yellow1 - self1);
	std::cout << "Blue 1: " << distanceToBlue1 << std::endl;
	std::cout << "Yellow 1: " << distanceToYellow1 << std::endl;
	double distanceToBlue2 = cv::norm(blue2 - self2);
	double distanceToYellow2 = cv::norm(yellow2 - self2);
	std::cout << "Blue 2: " << distanceToBlue2 << std::endl;
	std::cout << "Yellow 2: " << distanceToYellow2 << std::endl;
	std::cout << "======begin copy/paste============" << std::endl;
	std::cout << distanceToBlue2 << "=" << distanceToBlue1 << std::endl;
	std::cout << distanceToYellow2 << "=" << distanceToYellow1 << std::endl;
	std::cout << "======end copy/paste============" << std::endl;
	*/

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

