#include "BallLocations.h"


const void BallLocations::updateAndFilterClosest(cv::Point2i possibleClosestRaw, std::vector<cv::Point2i> rawBallCoords, bool ballIsNotValid, bool filterEnabled) {
	
	assert(false); // predict field coords
	if (!filterEnabled) {
		closest.updateCoordinates(possibleClosestRaw, cv::Point(0,0)); //Update all coordinates after filtering raw ones
		closest.lastRawCoords = possibleClosestRaw;
		closest.isUpdated = true;
		return;
	}

	if (reset == true || ballIsNotValid) {
		closest.filteredRawCoords = possibleClosestRaw;
		closest.lastRawCoords = possibleClosestRaw;
		closest.rawPixelCoords = possibleClosestRaw;
	}

	double distance = cv::norm(possibleClosestRaw - closest.lastRawCoords);
	if (distance > 40) { // another ball found
						 //detect, if correct ball is in vector
		bool foundFromVector = false;
		for (auto rawBallCoord : rawBallCoords) { //Sorted
			if (cv::norm(rawBallCoord - closest.lastRawCoords) <= 50) {
				possibleClosestRaw = rawBallCoord;
				foundFromVector = true;
				break;
			}
		}

		if (!foundFromVector) {
			double t2 = (double)cv::getTickCount();
			double dt = (t2 - ballLost) / cv::getTickFrequency();
			if (dt < 1) {
				closest.predictCoords();
				//closest.filteredRawCoords = closest.lastRawCoords;
				closest.rawPixelCoords = possibleClosestRaw;
				assert(false); //fix polar coords
				closest.updateCoordinates(closest.filteredRawCoords, cv::Point(0,0));
				closest.lastRawCoords = closest.filteredRawCoords;
				return;
			}
			else {
				reset = true;
			}
		}
	}

	closest.rawPixelCoords = possibleClosestRaw; //Filter needs the raw coords to be set
	closest.filterCoords(closest, reset); //Sets filtered raw coords
	assert(false); //fix polar coords
	closest.updateCoordinates(closest.filteredRawCoords, cv::Point(0,0)); //Update all coordinates after filtering raw ones
	closest.lastRawCoords = possibleClosestRaw;
	if (reset) {
		reset = false;
	}
	closest.isUpdated = true;
	ballLost = (double)cv::getTickCount();
}

