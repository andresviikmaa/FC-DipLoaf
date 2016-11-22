#pragma once
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/Interfaces.h"
#include "../VisionModule/VisionInterfaces.h"
//#include "ConfigurableModule.h"
#include <boost/property_tree/ptree.hpp>
#include <atomic>
#include "../DisplayModule/Dialog.h"


class DistanceCalibrator : public Dialog, public IUIEventListener {

public:
	DistanceCalibrator(ICamera * pCamera);

	~DistanceCalibrator();
	virtual bool OnMouseEvent(int event, float x, float y, int flags, bool bMainArea);
	static double calculateDistance(double centreX, double centreY, double x, double y);
	void start();
	void removeListener();
	const static int VIEWING_DISTANCE = 210;
	const static int DISTANCE_CALIBRATOR_STEP = 10;
	const static int CONF_SIZE = VIEWING_DISTANCE / DISTANCE_CALIBRATOR_STEP;
	std::string counterValue;
	void Enable(bool enable);
	std::string message = "";
	virtual int Draw();
	virtual void Start() {};

protected:
	bool enabled = false;
	cv::Mat bestLabels, clustered, centers;
	void mouseClicked(int x, int y, int flags);
	void mouseClicked2(int x, int y, int flags);
	ICamera *m_pCamera;
	std::vector<std::tuple<cv::Point, cv::Point, std::string>> points;
	std::vector<std::tuple<cv::Point, cv::Point, std::string>>::iterator itPoints;
	std::vector<cv::Vec3f> objectPoints;
	std::vector<cv::Vec3f> imagePoints;

	void calculateDistances();
private:
	cv::Point frame_size;
	boost::property_tree::ptree pt;
	int counter;
	cv::Mat frameBGR, frameHSV, buffer;

};