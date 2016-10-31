#pragma once
#include "BallPosition.h"

class BallLocations {
public:
	BallLocations(unsigned ballCount){
		reset = true;
		balls.resize(ballCount);
		// distribute balls uniformly
		for (unsigned i = 0; i < ballCount; i++) {
			balls[i].id = i;
		}
	}
	BallLocation& operator[](unsigned j) {
		return balls[j];
	}
	std::vector<BallLocation>::iterator begin() {
		return balls.begin();
	}
	std::vector<BallLocation>::iterator end() {
		return balls.end();
	}
	const BallLocation& getClosest() {
		return closest;
	}
	const void updateAndFilterClosest(cv::Point2i closestRaw, std::vector<cv::Point2i> rawBallCoords, bool ballIsNotValid, bool filterEnabled);

	size_t size() {
		return balls.size();
	}
public:
	BallLocation closest = BallLocation(true);
private:
	std::vector<BallLocation> balls;
	double ballLost = -1;
	bool reset;
};