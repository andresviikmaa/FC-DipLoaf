#pragma once
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/Interfaces.h"
#include "../VisionModule/VisionInterfaces.h"
#include "../DisplayModule/Dialog.h"

//#include "ConfigurableModule.h"
#include <atomic>
#include <opencv2/imgproc.hpp>
class AutoCalibrator: public Dialog, public IUIEventListener{
	enum {
		LIVE_FEED = 0,
		GRAB_FRAME,
		CALIBRATION,
		THRESHOLDING,
		GET_THRESHOLD,
		CROPPING
	};
protected:
	cv::Mat image;
	std::vector<std::pair<OBJECT, cv::Point>> markers;
public:
	AutoCalibrator(ICamera * pCamera);
	void Reset() { 
		//cv::resize(white, image, frame_size);
		//display = cv::Mat(frame_size.y, frame_size.x, CV_8U, cv::Scalar::all(255));
		//white.copyTo(display);
		frames = 0;
		screenshot_mode = LIVE_FEED;
	};
	HSVColorRange GetObjectThresholds(int index, const std::string &name);

	~AutoCalibrator();
	int frames = 0;
	virtual int Draw();
	virtual void Start() {
		assert(false);
	};

	const cv::Mat & GetFrame();
	virtual bool OnMouseEvent(int event, float x, float y, int flags);

protected:
	void mouseClicked(int x, int y, int flags);

	ICamera *m_pCamera;
	cv::Mat frameBGR, frameHSV, buffer;
	const cv::Mat white = cv::Mat(480, 640, CV_8UC3, cv::Scalar::all(245)); // blink display

	HSVColorRange range/* =  {{0,179},{0,255},{0,255}}*/;
	void SaveConf(const std::string &name);
private:
    bool done;
	std::string object_name;
	OBJECT object_id;
	cv::Point frame_size;
	int screenshot_mode;
	int last_screenshot_mode = -1;



};