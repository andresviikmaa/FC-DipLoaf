#pragma once
#include <opencv2/core.hpp>
#include "../CommonModule/Types.h"
#include <map>

typedef std::map<OBJECT, HSVColorRange> HSVColorRangeMap;
typedef std::map<OBJECT, cv::Mat> ThresholdedImages;

class ImageThresholder
{
protected:
	ThresholdedImages &thresholdedImages;
	HSVColorRangeMap &objectMap;
public:
	ImageThresholder(ThresholdedImages &images, HSVColorRangeMap &objectMap) : thresholdedImages(images), objectMap(objectMap) {};
	virtual void Start(cv::Mat &frameHSV, std::vector<OBJECT> objectList) = 0;
	virtual ~ImageThresholder() {};
};

