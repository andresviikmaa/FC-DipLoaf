#pragma once
#include "../CommonModule/Interfaces.h"
class DummyVision :
	public IVisionModule
{
protected:
	ICamera *m_pCamera;
	IDisplay *m_pDisplay;
	cv::Mat frameBGR;
public:
	DummyVision(ICamera * pCamera, IDisplay *pDisplay);
	virtual ~DummyVision();
	void ProcessFrame(double dt);
	//void Start();
	const cv::Mat & GetFrame() { return m_pCamera->Capture(); }
};

