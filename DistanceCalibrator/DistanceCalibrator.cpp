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
#include "opencv2/ccalib/omnidir.hpp"

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
	//points.push_back(std::make_tuple(cv::Point(0, 0), cv::Point(0, 0), std::string("robot location on field image")));

	createButton("Start", 'x', [&]{
		Enable(true);
	});
	createButton("Exit", 'x', [&]{
		stop_thread = true;
	});
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
		//if (bMainArea)
			mouseClicked((int)(x), (int)(y), flags);
		//else if (!bMainArea)
		//	mouseClicked2((int)(x), (int)(y), flags);
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
	std::vector<cv::Mat> objectPoints;
	std::vector<cv::Mat> imagePoints;
	size_t i = 0;
	auto it = points.rbegin();
	cv::Mat objects = cv::Mat(points.size(), 1, CV_64FC3);
	cv::Mat pixels = cv::Mat(points.size(), 1, CV_64FC2);
	cv::Vec3d *ptr = objects.ptr<cv::Vec3d>();
	cv::Vec2d *ptr2 = pixels.ptr<cv::Vec2d>();
	for (it++; it != points.rend(); it++){
		ptr[i] = cv::Vec3d(std::get<0>(*it).x, std::get<0>(*it).y, 0);
		ptr2[i] = cv::Vec2d(std::get<0>(*it).x, std::get<0>(*it).y);
		i++;

	}
	objectPoints.push_back(objects);
	imagePoints.push_back(pixels);

	std::vector<std::string> image_list, detec_list;
	int flags = 0;
	cv::Mat K, D, xi, idx;
	std::vector<cv::Vec3d> rvecs, tvecs;
	double _xi, rms;
	cv::TermCriteria criteria(3, 200, 1e-8);
	cv::InputArrayOfArrays objectPoints2(objectPoints);
	//std::cout << (objectPoints2.type() == CV_64FC3 ? "1" : "0") << std::endl;;
	CV_Assert(!objectPoints2.empty() && objectPoints2.type() == CV_64FC3);
	rms = cv::omnidir::calibrate(objectPoints, imagePoints, frameBGR.size(), K, xi, D, rvecs, tvecs, flags, criteria, idx);
	_xi = xi.at<double>(0);
	saveCameraParams("conf/"+m_pCamera->getName() +"/params.xml", flags, K, D, _xi,
		rvecs, tvecs, detec_list, idx, rms, imagePoints);

}

void DistanceCalibrator::saveCameraParams(const std::string & filename, int flags, const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs, const double xi, const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs,
	std::vector<std::string> detec_list, const cv::Mat& idx, const double rms, const std::vector<cv::Mat>& imagePoints)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty())
		fs << "nFrames" << (int)rvecs.size();

	if (flags != 0)
	{
		sprintf(buf, "flags: %s%s%s%s%s%s%s%s%s",
			flags & cv::omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
			flags & cv::omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
			flags & cv::omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
			flags & cv::omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
			flags & cv::omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
			flags & cv::omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
			flags & cv::omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
			flags & cv::omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
			flags & cv::omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "xi" << xi;

	//cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
	fs << "used_imgs" << "[";
	for (int i = 0; i < (int)idx.total(); ++i)
	{
		fs << detec_list[(int)idx.at<int>(i)];
	}
	fs << "]";

	if (!rvecs.empty() && !tvecs.empty())
	{
		cv::Mat rvec_tvec((int)rvecs.size(), 6, CV_64F);
		for (int i = 0; i < (int)rvecs.size(); ++i)
		{
			cv::Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(cv::Rect(0, i, 3, 1)));
			cv::Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(cv::Rect(3, i, 3, 1)));
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << rvec_tvec;
	}

	fs << "rms" << rms;
	
	if (!imagePoints.empty())
	{
		cv::Mat imageMat((int)imagePoints.size(), (int)imagePoints[0].total(), CV_64FC2);
		for (int i = 0; i < (int)imagePoints.size(); ++i)
		{
			cv::Mat r = imageMat.row(i).reshape(2, imageMat.cols);
			cv::Mat imagei(imagePoints[i]);
			imagei.copyTo(r);
		}
		fs << "image_points" << imageMat;
	}
	
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

