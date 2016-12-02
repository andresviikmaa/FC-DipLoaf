#include "FrontCameraVision.h"

extern FieldState gFieldState;


FrontCameraVision::FrontCameraVision(ICamera * pCamera): MainCameraVision(pCamera, "FrontCameraVision")
{

}


FrontCameraVision::~FrontCameraVision()
{
}

void  FrontCameraVision::ProcessFrame(double dt) {
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
	MainCameraVision::FindGates();
}
void FrontCameraVision::FindBall() {
	std::vector<cv::Point2d> balls;
	bool ballsFound = ballFinder.Locate(thresholdedImages[BALL], frameHSV, frameBGR, balls, 500);
	localState.ballCount = 0;
	for (auto ball : balls) {
		// this is dangerous as fixed size array is used. TODO: convert balls back to vector perhaps.
		//if (ballFinder.validateBall(thresholdedImages, ball, frameHSV, frameBGR)) {
			UpdateObjectPostion(lastBalls[localState.ballCount], ball);
			localState.ballCount++;
		//}
		if (localState.ballCount >= MAX_BALLS) break;
	}
	//localState.closestBallTribbler = localState.balls[0].isValid ? 0 : 15;
}

void FrontCameraVision::ResetUpdateState(){
	// reset all
	for (size_t i = 0; i < MAX_BALLS; i++) {
		lastBalls[i].isValid = false;
		lastBalls[i].isUpdated = false;
		lastBalls[i].isPredicted = false;
		lastBalls[i].distance = 10001;
		lastBalls[i].heading = 0;
		lastBalls[i].angle = 0;

	}

}
void FrontCameraVision::UpdateObjectPostion(ObjectPosition & object, const cv::Point2d &pos) {
	object.rawPixelCoords = pos - frameCenter;
	if (pos.x < 0) {
		object.isValid = false;
		return;
	}

	//Calculating distance
	double angle = (Vfov * (pos.y - frameCenter.y) / frameCenter.y) + CamAngleDev;
	double distance = CamHeight / tan(angle * PI / 180);
	//Calculating horizontal deviation
	double hor_space = tan(Hfov)*distance;
	double HorizontalDev = (hor_space * (pos.x - frameCenter.x) / frameCenter.x);
	double Hor_angle = atan(HorizontalDev / distance) * 180. / PI;
	/*
	if (Hor_angle > 0){
	Hor_angle = 360 - Hor_angle;
	}
	Hor_angle = abs(Hor_angle);
	*/
	object.polarMetricCoords = { distance, 360-Hor_angle };
	SYNC_OBJECT(object);
//	return{ distance, HorizontalDev, Hor_angle };
	object.isValid = true;
}

void FrontCameraVision::LoadSettings() {
	ADD_INT_SETTING(Vfov);
	ADD_INT_SETTING(CamHeight);
	ADD_INT_SETTING(CamAngleDev);
	MainCameraVision::LoadSettings();
}

bool FrontCameraVision::PublishState() {
	boost::mutex::scoped_lock lock(state_mutex); //allow one command at a time
	if (stateUpdated) {
		memcpy(&gFieldState.ballsFront, &localStateCopy.balls, MAX_BALLS * sizeof(BallPosition));
		gFieldState.closestBallTribbler = gFieldState.ballsFront[0].isValid ? 0 : MAX_BALLS-1;
		stateUpdated = false;

		return true;
	}
	return false;
};
