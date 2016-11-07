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
public:
	FrontCameraVision(ICamera * pCamera);
	~FrontCameraVision();
	void ProcessFrame();
	void ThresholdFrame();
	void FindGate();
	void FindBall();
};

