#include "AutoCalibrator.h"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#ifdef WIN32
#include <direct.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <cmath>

extern std::map<OBJECT, std::string> OBJECT_LABELS;

AutoCalibrator::AutoCalibrator(ICamera * pCamera) :Dialog("Color Calibrator", pCamera->GetFrameSize(), pCamera->GetFrameSize())
{
	range = { { 179, 0 }, { 255, 0 }, { 255, 0 } };
	m_pCamera = pCamera;
	frame_size = m_pCamera->GetFrameSize();
	AddEventListener(this);
	screenshot_mode = LIVE_FEED;
//	reset();
//	Start();
};

const cv::Mat & AutoCalibrator::GetFrame() {
	return m_pCamera->Capture(); 
}

HSVColorRange AutoCalibrator::GetObjectThresholds (int index, const std::string &name)
{

	//try {
	//	m_pCamera->GetObjectThresholds(index, name);
	//}
	//catch (...) {
	//}
	// order important, change name before state 
	this->object_name = name;
	screenshot_mode = GET_THRESHOLD;
	return range;


	/*
    cv::imshow(name.c_str(), image); //show the thresholded image
	cv::moveWindow(name.c_str(), 0, 0);
    cv::setMouseCallback(name.c_str(), [](int event, int x, int y, int flags, void* self) {
        if (event==cv::EVENT_LBUTTONUP) {
			((AutoCalibrator*)self)->mouseClicked(x, y, flags);
        }
		if (event == cv::EVENT_RBUTTONUP) {
			((AutoCalibrator*)self)->done = true;
		}
    }, this);


    done = false;
    while (!done)
    {
		std::this_thread::sleep_for(std::chrono::milliseconds(30)); // do not poll serial to fast
		
        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            std::cout << "esc key is pressed by user" << std::endl;
            done = true;
        }
	
    }
    cvDestroyWindow(name.c_str());
	*/
    SaveConf(object_name);
	screenshot_mode = THRESHOLDING;
    return range;

};
void AutoCalibrator::SaveConf(const std::string &name) {
	using boost::property_tree::ptree;

	ptree pt;
	pt.put("hue.low", range.hue.low);
	pt.put("hue.high", range.hue.high);
	pt.put("sat.low", range.sat.low);
	pt.put("sat.high", range.sat.high);
	pt.put("val.low", range.val.low);
	pt.put("val.high", range.val.high);
#ifdef WIN32
	_mkdir((std::string("conf/") + m_pCamera->getName()).c_str());
#else
	mkdir((std::string("conf/") + m_pCamera->getName()).c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
#endif
	write_ini(std::string("conf/") + m_pCamera->getName() + "/" + name + ".ini", pt);
}
bool AutoCalibrator::OnMouseEvent(int event, float x, float y, int flags) {
	if (screenshot_mode == CROPPING){
		return true;
	}else if (screenshot_mode == GET_THRESHOLD){
		if (event == cv::EVENT_LBUTTONUP) {
			mouseClicked((int)(x), (int)(y), flags);
		}
		if (event == cv::EVENT_RBUTTONUP) {
			SaveConf(this->object_name);
			last_screenshot_mode = screenshot_mode;
			screenshot_mode = THRESHOLDING;
		}
		return true;
	}
	return false;

};

void AutoCalibrator::mouseClicked(int x, int y, int flags){
	typedef std::vector<std::pair<cv::Point, cv::Point>> pointlist;
	//void ExamineNeigbourhood(pointlist points){
	//}
	markers.push_back(std::make_pair(object_id, cv::Point(x, y)));
	std::queue<cv::Point> points;

	int step = 1;
	if (object_id == FIELD) step = 10;
	points.push(cv::Point(x, y));
	std::vector<cv::Point> neighbourhood;
	neighbourhood.push_back(cv::Point(0, -step));
	neighbourhood.push_back(cv::Point(0, step));
	//
	neighbourhood.push_back(cv::Point(-step, 0));
	neighbourhood.push_back(cv::Point(step, 0));
	//
	//
	neighbourhood.push_back(cv::Point(-step, -step));
	neighbourhood.push_back(cv::Point(step, step));
	neighbourhood.push_back(cv::Point(step, -step));
	neighbourhood.push_back(cv::Point(-step, step));

	cv::Mat1b mask(frameHSV.rows, frameHSV.cols, uchar(0));
	mask.at<uchar>(y, x) = uchar(255);

	range = { { 179, 0 }, { 255, 0 }, { 255, 0 } };

	size_t counter = 0;
	cv::Vec3f hsv1;
	cv::Scalar mean;
	size_t N = neighbourhood.size();
	do {
		auto point = points.front();

		//cv::Vec3f hsv1 = frameHSV.at<cv::Vec3b>(point.y, point.x);
		mean = cv::mean(frameHSV, mask);
		hsv1 = cv::Vec3f(mean[0], mean[1], mean[2]);
		//for (auto n : neighbourhood){
		for (int i = 0; i < 3;i++){
			auto n = neighbourhood[rand() % N];
			auto next = point + n;
			cv::Vec3f hsv2 = frameHSV.at<cv::Vec3b>(next.y, next.x);
			auto col = frameBGR.at<cv::Vec3b>(next.y, next.x);
			if (col[2] == 255) continue;

			cv::Vec3f diff;
			diff[0] = abs(hsv1[0] - hsv2[0]);
			diff[1] = abs(hsv1[1] - hsv2[1]);
			diff[2] = abs(hsv1[2] - hsv2[2]);

			float error[3] = { 10, 20, 50 };
			if (object_id == BALL) { error[0] = 10; error[1] = 50; error[2] = 255; };
			if (object_id == FIELD) { error[0] = 10; error[1] = 30; error[2] = 255; };

			frameBGR.at<cv::Vec3b>(next.y, next.x) = cv::Vec3b(0, 0, 255);
			if (diff[0] < error[0] && diff[1] < error[1] && diff[2] < error[2]){
				//buffer.at<cv::Vec3b>(next.y, next.x) = cv::Vec3b(0, 0, 255);
				mask.at<uchar>(next.y, next.x) = uchar(255);
				points.push(next);
				if (hsv2[0] < range.hue.low)   range.hue.low = hsv2[0];
				if (hsv2[0] > range.hue.high)  range.hue.high = hsv2[0];

				if (hsv2[1] < range.sat.low)   range.sat.low = hsv2[1];
				if (hsv2[1] > range.sat.high)  range.sat.high = hsv2[1];

				if (hsv2[2] < range.val.low)   range.val.low = hsv2[2];
				if (hsv2[2] > range.val.high)  range.val.high = hsv2[2];

			}
		}
		points.pop();
		counter++;

	} while (!points.empty() && counter < 40000);
	std::cout << "counter: " << counter << ", mean: " << mean << std::endl;

	cv::Mat imgThresholded;
	cv::inRange(frameHSV, cv::Scalar(range.hue.low, range.sat.low, range.val.low), cv::Scalar(range.hue.high, range.sat.high, range.val.high), imgThresholded); //Threshold the image
	std::cout << cv::Scalar(range.hue.low, range.sat.low, range.val.low) << cv::Scalar(range.hue.high, range.sat.high, range.val.high) << std::endl;

	cv::Mat selected(imgThresholded.rows, imgThresholded.cols, CV_8U, cv::Scalar::all(0));

	buffer.copyTo(selected, 255 - imgThresholded);
	selected.copyTo(buffer);

}


AutoCalibrator::~AutoCalibrator(){
	RemoveEventListener(this);

}


int AutoCalibrator::Draw() {
	//cv::waitKey(0);

		if (screenshot_mode == LIVE_FEED){
			if (last_screenshot_mode != LIVE_FEED){
				clearButtons();
				createButton("Take screenshot", 'c', [&] {
					screenshot_mode = GRAB_FRAME;
				});
				createButton("Exit", 'x', [&]{
					stop_thread = true;
				});
			}
			frameBGR = m_pCamera->Capture();
			GaussianBlur(frameBGR, frameBGR, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

			frameBGR.copyTo(buffer);
			ShowImage(frameBGR);
		}

		else if (screenshot_mode == GRAB_FRAME){
			last_screenshot_mode = screenshot_mode;
			frameBGR.copyTo(image);
			cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
			int shift = 25; // in openCV hue values go from 0 to 180 (so have to be doubled to get to 0 .. 360) because of byte range from 0 to 255
			for (int j = 0; j<frameHSV.rows; ++j)
				for (int i = 0; i<frameHSV.cols; ++i)
				{
					frameHSV.at<unsigned char>(j, i) = (frameHSV.at<unsigned char>(j, i) + shift) % 180;
				}

			screenshot_mode = THRESHOLDING;

		}
		else if (screenshot_mode == THRESHOLDING){
			if (last_screenshot_mode != THRESHOLDING){
				clearButtons();
				for (int i = 0; i < NUMBER_OF_OBJECTS; i++) {
					createButton(OBJECT_LABELS[(OBJECT)i], '-', [&, i] {
						//GetObjectThresholds(i, OBJECT_LABELS[(OBJECT)i]);
						object_name = OBJECT_LABELS[(OBJECT)i];
						object_id = (OBJECT)i;
						screenshot_mode = GET_THRESHOLD;
					});

				}
				createButton("Back", 'r', [&] {
					Reset();
				});
			}

			last_screenshot_mode = screenshot_mode;
			image.copyTo(buffer);
			ShowImage(buffer);
		}
		else if (screenshot_mode == GET_THRESHOLD) {
			if (last_screenshot_mode != GET_THRESHOLD){
				clearButtons();
			}
			cv::putText(buffer, object_name, cv::Point((int)(image.cols * 0.3), (int)(image.rows*0.3)), cv::FONT_HERSHEY_DUPLEX , 1, cv::Scalar(23, 67, 245));
			cv::putText(buffer, "(ctrl +) click to select pixels, right click back", cv::Point((int)(image.cols * 0.2), (int)(image.rows*0.5)), cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(23, 67, 245));
			ShowImage(buffer);
			last_screenshot_mode = screenshot_mode;

		}
		return Dialog::Draw();
}

