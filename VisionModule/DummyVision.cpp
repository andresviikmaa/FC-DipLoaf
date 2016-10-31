#include "DummyVision.h"



DummyVision::DummyVision(ICamera * pCamera, IDisplay *pDisplay)
{
	m_pCamera = pCamera;
	m_pDisplay = pDisplay;
}


DummyVision::~DummyVision()
{
}

void DummyVision::ProcessFrame(double dt) {
	frameBGR = m_pCamera->Capture();
	m_pDisplay->ShowImage(frameBGR);
}