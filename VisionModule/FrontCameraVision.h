#pragma once
#include "../CommonModule/Types.h"
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/ConfigurableModule.h"
#include "VisionInterfaces.h"
#include "../CommonModule/FieldState.h"
#include "TBBImageThresholder.h"

class FrontCameraVision :
	public ConfigurableModule, public IVisionModule
{
protected:
	ICamera *m_pCamera;
	IDisplay *m_pDisplay;

	cv::Mat frameBGR;
	cv::Mat frameHSV;
	HSVColorRangeMap objectThresholds;
public:
	FrontCameraVision(ICamera * pCamera, IDisplay *pDisplay);
	~FrontCameraVision();

	virtual void ProcessFrame(double dt);
	const cv::Mat & GetFrame() { return m_pCamera->Capture(); }
	virtual ICamera * GetCamera() { return m_pCamera; };

private:
	bool hideUseless = false;

	FieldState localState;
	ThresholdedImages thresholdedImages;
	TBBImageThresholder thresholder;
};

