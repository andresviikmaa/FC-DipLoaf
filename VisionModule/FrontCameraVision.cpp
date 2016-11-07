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
	FindGate();
	FindBall();

}
void FrontCameraVision::ThresholdFrame() {
	thresholder.Start(frameHSV, { BALL, gFieldState.targetGate });
}

void FrontCameraVision::FindGate() {
}
void FrontCameraVision::FindBall() {
}