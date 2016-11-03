#include "FrontCameraVision.h"



FrontCameraVision::FrontCameraVision(ICamera * pCamera, IDisplay *pDisplay): ConfigurableModule("FrontCameraVision")
	, thresholder(thresholdedImages, objectThresholds)
{
	m_pCamera = pCamera;
	m_pDisplay = pDisplay;

	//	videoRecorder = new VideoRecorder("videos/", 30, m_pCamera->GetFrameSize(true));
	LoadSettings();
	//Start();
}


FrontCameraVision::~FrontCameraVision()
{
}


void  FrontCameraVision::ProcessFrame(double dt) {
	return;
	frameBGR = m_pCamera->Capture();


	if (!hideUseless) {
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(0, -30), (frameSize / 2) + cv::Size(0, 30), cv::Scalar(0, 0, 255), 3, 8, 0);
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(-30, 0), (frameSize / 2) + cv::Size(30, 0), cv::Scalar(0, 0, 255), 3, 8, 0);
		m_pDisplay->ShowImage(frameBGR);
	}

}