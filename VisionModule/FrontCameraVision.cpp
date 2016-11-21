#include "FrontCameraVision.h"

extern FieldState gFieldState;


FrontCameraVision::FrontCameraVision(ICamera * pCamera): MainCameraVision(pCamera, "FrontCameraVision")
{

}


FrontCameraVision::~FrontCameraVision()
{
}

void  FrontCameraVision::ProcessFrame() {
	ThresholdFrame();
	//FindGate();
	FindBall();

}
//void FrontCameraVision::ThresholdFrame() {
//	if (thresholder == nullptr)
//		thresholder = new TBBImageThresholder(thresholdedImages, objectThresholds);
//	thresholder->Start(frameHSV, { BALL, gFieldState.targetGate });
//}

void FrontCameraVision::FindGate() {

}
void FrontCameraVision::FindBall() {

}

void FrontCameraVision::UpdateObjectPostion(ObjectPosition & object, const cv::Point2d &pos) {
	object.rawPixelCoords = pos - cameraOrgin;
	if (pos.x < 0) {
		object.isValid = false;
		return;
	}

	//Calculating distance
	double angle = (Vfov * (pos.y - cameraOrgin.y) / cameraOrgin.y) + CamAngleDev;
	double distance = CamHeight / tan(angle * PI / 180);
	//Calculating horizontal deviation
	double hor_space = tan(Hfov)*distance;
	double HorizontalDev = (hor_space * (pos.x - cameraOrgin.x) / cameraOrgin.x);
	double Hor_angle = atan(HorizontalDev / distance) * 180. / PI;
	/*
	if (Hor_angle > 0){
	Hor_angle = 360 - Hor_angle;
	}
	Hor_angle = abs(Hor_angle);
	*/
	object.polarMetricCoords = { distance, Hor_angle };
	SYNC_OBJECT(object);
//	return{ distance, HorizontalDev, Hor_angle };

}

void FrontCameraVision::LoadSettings() {
	ADD_INT_SETTING(Vfov);
	ADD_INT_SETTING(CamHeight);
	ADD_INT_SETTING(CamAngleDev);
	MainCameraVision::LoadSettings();
}

void FrontCameraVision::PublishState() {
	//gFieldState.ballInFront;
};
