#pragma once
#include "../CommonModule/Types.h"
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/ConfigurableModule.h"
#include "VisionInterfaces.h"
#include "../CommonModule/FieldState.h"
#include "MainCameraVision.h"

class FrontCameraVision :
	public MainCameraVision
{
protected:
	double Hfov = 35.21;
	double Vfov = 21.65; //half of cameras vertical field of view (degrees)
	double CamHeight = 345; //cameras height from ground (mm)
	double CamAngleDev = 26; //deviation from 90* between ground
	ObjectPosition ball;
	ObjectPosition ball_copy;
public:
	FrontCameraVision(ICamera * pCamera);
	~FrontCameraVision();
	void ProcessFrame();
//	void ThresholdFrame();
	void FindGate();
	void FindBall();

	void UpdateObjectPostion(ObjectPosition & object, const cv::Point2d &pos);
	void LoadSettings();
	void PublishState();

};

