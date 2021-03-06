#pragma once
#include "VisionInterfaces.h"

class SimpleImageThresholder :
	public ImageThresholder
{
public:
	SimpleImageThresholder(ThresholdedImages &images, HSVColorRangeMap &objectMap);
	~SimpleImageThresholder();
	void Start(cv::Mat &frameHSV, std::vector<OBJECT> objectList) {
		for (auto &object : objectList) {
			auto r = objectMap[object];
			inRange(frameHSV, cv::Scalar(r.hue.low, r.sat.low, r.val.low), cv::Scalar(r.hue.high, r.sat.high, r.val.high), thresholdedImages[object]);
		}
	}

};

