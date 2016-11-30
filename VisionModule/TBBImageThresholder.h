#pragma once
#include "../CommonModule/Types.h"
#include "VisionInterfaces.h"

//#define USE_INRANGE
class TBBImageThresholder :
	public ImageThresholder, public cv::ParallelLoopBody
{
public:
	TBBImageThresholder(ThresholdedImages &images, HSVColorRangeMap &objectMap) : ImageThresholder(images, objectMap){
	}

	void Start(cv::Mat &frameHSV, std::vector<OBJECT> objectList) {
		this->frameHSV = frameHSV;
		this->objectList = objectList;

		cv::parallel_for_(cv::Range(0, diff), *this);
		/*
		cv::extractChannel(frameHSV, thresholdedImages[BALL], 0);
		cv::extractChannel(frameHSV, thresholdedImages[BLUE_GATE], 1);
		cv::extractChannel(frameHSV, thresholdedImages[YELLOW_GATE], 2);
		*/
	}

	virtual void operator()(const cv::Range& range) const
	{
		auto &rbl = objectMap[BALL];
		auto &rbg = objectMap[BLUE_GATE];
		auto &ryg = objectMap[YELLOW_GATE];

		auto &rfd = objectMap[FIELD];
		auto &rib = objectMap[INNER_BORDER];
		auto &rob = objectMap[OUTER_BORDER];

		auto &tbl = thresholdedImages[BALL];
		auto &tbg = thresholdedImages[BLUE_GATE];
		auto &tyg = thresholdedImages[YELLOW_GATE];

		auto &tfd = thresholdedImages[FIELD];
		auto &tib = thresholdedImages[INNER_BORDER];
		auto &tob = thresholdedImages[OUTER_BORDER];

		for (int i = range.start; i < range.end; i++)
		{

#define THRESHOLD_HSV(range, h, s,v) \
					(range.hue.low <= h) && (range.hue.high >= h) && (range.sat.low <= s) && (range.sat.high >= s) && (range.val.low <= v) && (range.val.high >= v)
#define THRESHOLD_HS(range, h, s,v) \
					(range.hue.low <= h) && (range.hue.high >= h) && (range.sat.low <= s) && (range.sat.high >= s)

#define THRESHOLD THRESHOLD_HSV

#ifndef USE_INRANGE
			for (int j = (frameHSV.cols*frameHSV.rows * 3 / diff)*i, k = (frameHSV.cols*frameHSV.rows / diff)*i; j < (frameHSV.cols*frameHSV.rows * 3 / diff)*(i + 1); j += 3, k++) {
				//if (j % 2) continue;
				int h = frameHSV.data[j];
				int s = frameHSV.data[j + 1];
				int v = frameHSV.data[j + 2];

				bool ball = THRESHOLD(rbl, h, s, v);
				bool blue = THRESHOLD(rbg, h, s, v);
				bool yellow = THRESHOLD(ryg, h, s, v);

				bool field = THRESHOLD(rfd, h, s, v);
				bool inner_b = THRESHOLD(rib, h, s, v);
				bool outer_b = THRESHOLD(rob, h, s, v);



				frameHSV.data[j] = h * 179;
				frameHSV.data[j + 1] = s * 255;
				frameHSV.data[j + 2] = v * 255;


				tfd.data[k] = field || inner_b ? 255 : 0;

				tbl.data[k] = ball && !yellow ? 255 : 0;
				tbg.data[k] = blue && !field ? 255 : 0;
				tyg.data[k] = yellow ? 255 : 0;

				tib.data[k] = inner_b ? 255 : 0;
				tob.data[k] = outer_b ? 255 : 0;



			}


#else // use inRange
			cv::Mat in(frameHSV, cv::Rect(0, (frameHSV.rows / diff)*i,
				frameHSV.cols, frameHSV.rows / diff));

			for (auto &object : objectList) {
				auto r = objectMap[object];
				cv::Mat out(thresholdedImages[object], cv::Rect(0, (thresholdedImages[object].rows / diff)*i,
					thresholdedImages[object].cols, thresholdedImages[object].rows / diff));
				inRange(in, cv::Scalar(r.hue.low, r.sat.low, r.val.low), cv::Scalar(r.hue.high, r.sat.high, r.val.high), out);
			}
#endif
		}
	}
protected:
	cv::Mat frameHSV;
	std::vector<OBJECT> objectList;
	int diff = 4;
};
