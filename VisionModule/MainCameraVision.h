#pragma once
#include "../CommonModule/Types.h"
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/ConfigurableModule.h"
#include "VisionInterfaces.h"
#include "../CommonModule/FieldState.h"
#include "GateFinder.h"
#include "BallFinder.h"
#include "RobotFinder.h"
//#include "SimpleImageThresholder.h"
//#include "ThreadedImageThresholder.h"
//#include "ParallelImageThresholder.h"
#include "TBBImageThresholder.h"

#define SYNC_OBJECT(object) object.distance =  object.polarMetricCoords.x; \
object.angle = object.polarMetricCoords.y;	\
if (object.angle> 0) \
	object.heading = object.angle > 180 ? object.angle - 360 : object.angle; \
else \
	object.heading = object.angle < -180 ? object.angle + 360 : object.angle; 



class VideoRecorder;
class MainCameraVision :
	public ConfigurableModule, public IVisionModule, public ThreadedClass
{
protected:
	ICamera *m_pCamera;
	IDisplay *m_pDisplay;

	cv::Mat frameBGR;
	cv::Mat frameHSV;
	HSVColorRangeMap objectThresholds;

	bool gaussianBlurEnabled = false;
	bool sonarsEnabled = false;
	bool greenAreaDetectionEnabled = false;
	bool gateObstructionDetectionEnabled = false;
	bool borderDetectionEnabled = true;
	bool borderCollisonEnabled = false;
	bool fieldCollisonEnabled = false;
	bool nightVisionEnabled = false;
	bool detectOtherRobots = false;
	bool detectObjectsNearBall = false;
	bool useKalmanFilter = false;

	VideoRecorder *videoRecorder  = NULL;
	double time = 0;

	//bool _collisionWithBorder;
	//bool _collisionWithUnknown;
	//bool _obstacleNearBall;
	//cv::Point2d _collisionRange; // which directions are blocked
	//bool _gateObstructed;
	bool _somethingOnWay;
	FieldState localState;
	FieldState localStateCopy;
	bool stateUpdated = false;
	bool m_bEnabled = false;

	GateFinder blueGateFinder;
	GateFinder yellowGateFinder;
	BallFinder ballFinder;
	RobotFinder robotFinder;

	void resetBallsUpdateState() {
		return;
		//for (size_t i = 0, isize = _balls.size(); i < isize; i++) {
		//	_balls[i].setIsUpdated(false);
		//}
	}
	std::vector<cv::Point2i> notBlueGates, notYellowGates;

	ThresholdedImages thresholdedImages;
	TBBImageThresholder *thresholder = nullptr;
	void ThresholdFrame();
	void CheckGateObstruction();
	void FindGates();
	void CheckCollisions();
	void FindBalls();
	void FindOtherRobots();
	cv::Point2d cameraOrgin;
	virtual void UpdateObjectPostion(ObjectPosition & object, const cv::Point2d &pos);
	std::vector<OBJECT> thresholdObjects;

public:
	boost::mutex state_mutex;
	MainCameraVision(ICamera * pCamera, const std::string sName="MainCameraVision");
	virtual ~MainCameraVision();
	virtual void ProcessFrame();
	bool PublishState();
	virtual void Enable(bool enable) {
		m_bEnabled = enable;
	}
	void Start();
	void Run();
	const cv::Mat & GetFrame() { return m_pCamera->Capture();  }
	bool captureFrames();
	void captureFrames(bool start);
	virtual ICamera * GetCamera() {
		return m_pCamera;
	};

};

