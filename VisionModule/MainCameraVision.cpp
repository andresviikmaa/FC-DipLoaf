#include "MainCameraVision.h"


#include <queue>          // std::priority_queue
#include <functional>     // std::greater
//#include "VideoRecorder.h"
#include "../CommonModule/FieldState.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

extern FieldState gFieldState;
//extern int number_of_balls;


double angleBetween(const cv::Point2d &a, const cv::Point2d &b) {
	double alpha = atan2(a.y, a.x) - atan2(b.y, b.x);
	double alphaDeg = alpha * 180. / CV_PI;
	if (alphaDeg < 0) alphaDeg += 360;
	return alphaDeg;
}

bool angleInRange(cv::Point2d point, cv::Point2d range) {
	double a1 = -angleBetween(point, cv::Point(0, -1)) + 360;
	if (range.x < range.y) {
		return range.x < a1 && range.y > a1;
	}
	else {
		return range.x > a1 && range.y < a1;

	}
}

MainCameraVision::MainCameraVision(ICamera *pCamera, IDisplay *pDisplay) : ConfigurableModule("MainCameraVision"), ThreadedClass("MainCameraVision")
, thresholder(thresholdedImages, objectThresholds)
{
	m_pCamera = pCamera;
	m_pDisplay = pDisplay;

	ADD_BOOL_SETTING(gaussianBlurEnabled);
	ADD_BOOL_SETTING(greenAreaDetectionEnabled);
	ADD_BOOL_SETTING(gateObstructionDetectionEnabled);
	ADD_BOOL_SETTING(borderDetectionEnabled);
	ADD_BOOL_SETTING(borderCollisonEnabled);
	ADD_BOOL_SETTING(fieldCollisonEnabled);
	ADD_BOOL_SETTING(nightVisionEnabled);
	ADD_BOOL_SETTING(detectOtherRobots);
	ADD_BOOL_SETTING(detectObjectsNearBall);
	ADD_BOOL_SETTING(hideUseless);
	ADD_BOOL_SETTING(useKalmanFilter);
//	videoRecorder = new VideoRecorder("videos/", 30, m_pCamera->GetFrameSize(true));
	LoadSettings();
	Start();
}


MainCameraVision::~MainCameraVision()
{
//	if (videoRecorder != NULL) {
//		videoRecorder->Stop();
//		delete videoRecorder;
//		videoRecorder = NULL;
//	}
}
bool MainCameraVision::captureFrames(){
//	return videoRecorder->isRecording;
	return false;
}

void MainCameraVision::captureFrames(bool start){
//	if (start) {
//		videoRecorder->Start();
//	}
//	else {
//		videoRecorder->Stop();
//	}
}
void MainCameraVision::Run() {
	while (!stop_thread) {
		frameBGR = m_pCamera->Capture();
		if (m_bEnabled) {
			ProcessFrame(0);
			{
				boost::mutex::scoped_lock lock(state_mutex); //allow one command at a time
				memcpy(&localStateCopy, &localState, sizeof(FieldState));
				stateUpdated = true;
			}
		}
		else {
			;//sleep
			Sleep(10);
		}
	}
}
void MainCameraVision::PublishState() {
	boost::mutex::scoped_lock lock(state_mutex); //allow one command at a time
	if (stateUpdated) {
		memcpy(&gFieldState, &localStateCopy, sizeof(FieldState));
		stateUpdated = false;
	}
}
void  MainCameraVision::ProcessFrame(double dt) {
	ThresholdFrame();
	CheckGateObstruction();
	FindGates(dt);
	CheckCollisions();
	FindBalls(dt);

	if (!hideUseless) {
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(0, -30), (frameSize / 2) + cv::Size(0, 30), cv::Scalar(0, 0, 255), 3, 8, 0);
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(-30, 0), (frameSize / 2) + cv::Size(30, 0), cv::Scalar(0, 0, 255), 3, 8, 0);
		m_pDisplay->ShowImage(frameBGR);
	}

}
void MainCameraVision::ThresholdFrame() {

	//		if (videoRecorder->isRecording){
	//			videoRecorder->RecordFrame(frameBGR, "");
	//		}
	/**************************************************/
	/*	STEP 1. Convert picture to HSV colorspace	  */
	/**************************************************/
	if (gaussianBlurEnabled) {
		cv::GaussianBlur(frameBGR, frameBGR, cv::Size(3, 3), 4);
	}
	cvtColor(frameBGR, frameHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	/**************************************************/
	/*	STEP 2. thresholding in parallel	          */
	/**************************************************/

	thresholder.Start(frameHSV, { BALL, BLUE_GATE, YELLOW_GATE, FIELD, INNER_BORDER, OUTER_BORDER });



}
void MainCameraVision::CheckGateObstruction() {
	if (gateObstructionDetectionEnabled) {

		/**************************************************/
		/*	STEP 3. check that path to gate is clean      */
		/* this is done here, because finding contures	  */
		/* corrupts thresholded imagees					  */
		/**************************************************/
		cv::Mat selected(frameBGR.rows, frameBGR.cols, CV_8U, cv::Scalar::all(0));
		cv::Mat mask(frameBGR.rows, frameBGR.cols, CV_8U, cv::Scalar::all(0));
		cv::Mat	tmp(frameBGR.rows, frameBGR.cols, CV_8U, cv::Scalar::all(0));
		//cv::line(mask, cv::Point(frameSize.width / 2, 200), cv::Point(frameSize.width / 2 - 40, frameSize.height - 100), cv::Scalar(255, 255, 255), 40);
		//cv::line(mask, cv::Point(frameSize.width / 2, 200), cv::Point(frameSize.width / 2 + 40, frameSize.height - 100), cv::Scalar(255, 255, 255), 40);
		std::vector<cv::Point2i> triangle;
		int halfWidth = frameBGR.cols / 2;
		int halfHeight = frameBGR.rows / 2;
		triangle.push_back(cv::Point(halfWidth, halfHeight - 45));
		triangle.push_back(cv::Point(halfWidth - 80, halfHeight / 2 - 45));
		triangle.push_back(cv::Point(halfWidth + 80, halfHeight / 2 - 45));
		cv::fillConvexPoly(mask, triangle, cv::Scalar::all(255));
		tmp = 255 - (thresholdedImages[INNER_BORDER] + thresholdedImages[OUTER_BORDER] + thresholdedImages[FIELD]);
		tmp.copyTo(selected, mask); // perhaps use field and inner border
		thresholdedImages[SIGHT_MASK] = selected;
		cv::polylines(frameBGR, triangle, true, (255, 255, 255));
		//sightObstructed = countNonZero(selected) > 10;
		//Gate obstruction

		// step 3.2
		int count = countNonZero(thresholdedImages[SIGHT_MASK]);
		std::cout << "obsCount: " << count << std::endl;
		localState.gateObstructed = count > 900;
		//cv::putText(thresholdedImages[SIGHT_MASK], osstr.str(), cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255));
		//cv::imshow("mmm", thresholdedImages[SIGHT_MASK]);
	}
	else {
		localState.gateObstructed = false;
	}
	if (greenAreaDetectionEnabled) {
		bool notEnoughtGreen = countNonZero(thresholdedImages[FIELD]) < 640 * 120;
		_somethingOnWay = notEnoughtGreen;
	}
	else {
		_somethingOnWay = false;
	}
}

void MainCameraVision::FindGates(double dt) {
	/**************************************************/
	/* STEP 4. extract closest ball and gate positions*/
	/**************************************************/
	cv::Point2f blueGate[4], yellowGate[4];
	cv::Point blueGateCenter, yellowGateCenter;

	//Blue gate pos
	bool blueFound = blueGateFinder.Locate(thresholdedImages[BLUE_GATE], frameHSV, frameBGR, blueGateCenter, blueGate, notBlueGates);
	localState.gates[BLUE_GATE].isValid = false;
	localState.gates[YELLOW_GATE].isValid = false;

	if (blueFound) {
		cv::Point vertices[4];
		for (int i = 0; i < 4; ++i) {
			vertices[i] = blueGate[i];
		}
		cv::fillConvexPoly(thresholdedImages[BALL], vertices, 4, cv::Scalar::all(0));
		m_pCamera->UpdateObjectPostion(localState.gates[BLUE_GATE], blueGateCenter);
		
	}

	//Yellow gate pos
	bool yellowFound = yellowGateFinder.Locate(thresholdedImages[YELLOW_GATE], frameHSV, frameBGR, yellowGateCenter, yellowGate, notYellowGates);
	if (yellowFound) {
		cv::Point vertices[4];
		for (int i = 0; i < 4; ++i) {
			vertices[i] = yellowGate[i];
		}
		cv::fillConvexPoly(thresholdedImages[BALL], vertices, 4, cv::Scalar::all(0));
		m_pCamera->UpdateObjectPostion(localState.gates[YELLOW_GATE], yellowGateCenter);

	}

	// ajust gate positions to ..
	// find closest points to opposite gate centre
	if (blueFound && yellowFound) {
		auto min_i1 = 0, min_j1 = 0, min_i2 = 0, min_j2 = 0;
		double min_dist1 = INT_MAX, min_dist2 = INT_MAX;
		for (int i = 0; i < 4; i++) {
			double dist1 = cv::norm(blueGate[i] - (cv::Point2f)yellowGateCenter);
			double dist2 = cv::norm(yellowGate[i] - (cv::Point2f)blueGateCenter);
			if (dist1 < min_dist1) {
				min_dist1 = dist1;
				min_i1 = i;
			}
			if (dist2 < min_dist2) {
				min_dist2 = dist2;
				min_j1 = i;
			}
		}
		auto next = (min_i1 + 1) % 4;
		auto prev = (min_i1 + 3) % 4;
		// find longest side
		min_i2 = (cv::norm(blueGate[min_i1] - blueGate[next]) > cv::norm(blueGate[min_i1] - blueGate[prev])) ? next : prev;
		next = (min_j1 + 1) % 4;
		prev = (min_j1 + 3) % 4;
		// find longest side
		min_j2 = (cv::norm(yellowGate[min_j1] - yellowGate[next]) > cv::norm(yellowGate[min_j1] - yellowGate[prev])) ? next : prev;
		cv::Scalar color4(0, 0, 0);

		cv::Scalar color2(0, 0, 255);

		cv::Point2d c1 = (blueGate[min_i1] + blueGate[min_i2]) / 2;


		cv::Point2d c2 = (yellowGate[min_j1] + yellowGate[min_j2]) / 2;

		if (!hideUseless) {
			circle(frameBGR, c2, 12, color2, -1, 8, 0);
			circle(frameBGR, c1, 12, color4, -1, 12, 0);
		}

		m_pCamera->UpdateObjectPostion(localState.gates[BLUE_GATE], c1);
		m_pCamera->UpdateObjectPostion(localState.gates[YELLOW_GATE], c2);


		//_blueGate.updateCoordinates(c1, m_pCamera->getPolarCoordinates(c1));
		//_yellowGate.updateCoordinates(c2, m_pCamera->getPolarCoordinates(c2));
		//
		//_self.updateFieldCoords(cv::Point2d(0, 0), dt);
	}
	//else {
	//	_self.predict(dt);
	//	// calculate gates from predicted pos.
	//	_blueGate.polarMetricCoords.x = cv::norm(_self.fieldCoords - _blueGate.fieldCoords);
	//	_blueGate.polarMetricCoords.y = 360 - angleBetween(cv::Point(0, 1), _self.fieldCoords - (_blueGate.fieldCoords)) + _self.getAngle();
	//	_yellowGate.polarMetricCoords.x = cv::norm(_self.fieldCoords - _yellowGate.fieldCoords);;
	//	_yellowGate.polarMetricCoords.y = 360 - angleBetween(cv::Point(0, 1), _self.fieldCoords - (_yellowGate.fieldCoords)) + _self.getAngle();
	//}

	cv::circle(thresholdedImages[FIELD], cv::Point(frameBGR.size() / 2), 70, 255, -1);
	cv::circle(thresholdedImages[OUTER_BORDER], cv::Point(frameBGR.size() / 2), 70, 0, -1);
	cv::circle(thresholdedImages[INNER_BORDER], cv::Point(frameBGR.size() / 2), 70, 0, -1);
	cv::circle(thresholdedImages[BALL], cv::Point(frameBGR.size() / 2), 50, 0, -1);

}
void MainCameraVision::FindBalls(double dt) {
	std::vector<cv::Point2d> balls;
	bool ballsFound = ballFinder.Locate(thresholdedImages[BALL], frameHSV, frameBGR, balls);
	localState.ballCount = 0;
	for (auto ball : balls) {
		m_pCamera->UpdateObjectPostion(localState.balls[localState.ballCount], ball);
		localState.ballCount++;
	}
	//if (localState.ballCount > 11) {
	//	cv::imshow("err", frameBGR);
	//	cv::waitKey(0);
	//}

}
void MainCameraVision::FindOtherRobots(double dt) {
	
	if (detectOtherRobots) {

		std::vector<cv::Point2i> robots;
		cv::bitwise_or(thresholdedImages[OUTER_BORDER], thresholdedImages[FIELD], thresholdedImages[FIELD]);
		bool robotsFound = robotFinder.Locate(thresholdedImages[FIELD], frameHSV, frameBGR, robots);
		if (!hideUseless) {
			for (auto robot : robots) {
				cv::Rect robotRectangle = cv::Rect(robot - cv::Point(20, 20) + cv::Point(frameBGR.size() / 2),
					robot + cv::Point(20, 20) + cv::Point(frameBGR.size() / 2));
				rectangle(frameBGR, robotRectangle.tl(), robotRectangle.br(), cv::Scalar(10, 255, 101), 2, 8, 0);
			}
		}
		bool ourRobotBlueBottom = (gFieldState.robotColor == ROBOT_COLOR_YELLOW_UP);
		//std::vector<cv::Point2d> robots;
		//bool ballsFound = ballFinder.Locate(thresholdedImages[FIELD], frameHSV, frameBGR, robots);

		std::vector<std::pair<cv::Point2i, double>> positionsToDistances; //One of the colors position and according distances
		for (size_t blueIndex = 0; blueIndex < notBlueGates.size(); blueIndex++) {
			for (size_t yellowIndex = 0; yellowIndex < notYellowGates.size(); yellowIndex++) {
				cv::Point2i bluePos = notBlueGates[blueIndex];
				cv::Point2i yellowPos = notYellowGates[yellowIndex];
				double distBetweenYellowBlue = cv::norm(bluePos - yellowPos);
				cv::Point2i frameCenter = cv::Point2i(frameBGR.cols / 2, frameBGR.rows / 2); //our robot is in center
				double distBetweenBlueAndRobot = cv::norm(bluePos - frameCenter);
				double distBetweenYellowAndRobot = cv::norm(yellowPos - frameCenter);

				if (distBetweenYellowBlue < 200 &&   //If distance between two colors is great, then it cannot be robot
													 //If our robot has bottom blue, then blue distance from robot has to less than yellow
					((ourRobotBlueBottom && distBetweenBlueAndRobot < distBetweenYellowAndRobot)
						//If our robot has not bottom blue, then blue distance from robot has to be greater than yellow
						|| (!ourRobotBlueBottom && distBetweenBlueAndRobot > distBetweenYellowAndRobot))) {
					std::pair<cv::Point2i, double> positionToDistance = std::make_pair(bluePos, distBetweenYellowBlue);
					positionsToDistances.push_back(positionToDistance);
				}
			}
		}
		auto sortFunc = [](std::pair<cv::Point2i, double> posToDis1, std::pair<cv::Point2i, double> posToDis2) { return (posToDis1.second < posToDis2.second); };
		std::sort(positionsToDistances.begin(), positionsToDistances.end(), sortFunc);
		if (positionsToDistances.size() > 0) {
			m_pCamera->UpdateObjectPostion(localState.partner, positionsToDistances[0].first);
		}
	}
}
void MainCameraVision::CheckCollisions() {
	if (borderCollisonEnabled || fieldCollisonEnabled) {
		bool wasCollisionWithBorder = localState.collisionWithBorder;
	bool wasCollisionWithUnknown = localState.collisionWithUnknown;
	// mask ourself
	//cv::circle(frameBGR, cv::Point(frameBGR.size() / 2), 70, 255, -1);
	cv::bitwise_or(thresholdedImages[INNER_BORDER], thresholdedImages[FIELD], thresholdedImages[FIELD]);
	//			cv::bitwise_or(thresholdedImages[FIELD], thresholdedImages[BALL], thresholdedImages[FIELD]);
	//			cv::bitwise_or(thresholdedImages[FIELD], thresholdedImages[BLUE_GATE], thresholdedImages[FIELD]);
	//			cv::bitwise_or(thresholdedImages[FIELD], thresholdedImages[YELLOW_GATE], thresholdedImages[FIELD]);
	//imshow("a", thresholdedImages[FIELD]);
	//cv::waitKey(1);
	localState.collisionRange = { -1, -1 };
	bool collisionWithBorder = false;
	bool collisonWithUnknown = false;

	for (size_t c/*orner*/ = 0; c < 4; c++) {
		cv::Rect privateZone(0, 0, 100, 100);

		//if (c == 0) privateZone = cv::Rect (0, 0, 100, 100); //c==0
		//else if (c == 1) privateZone = cv::Rect (0, -100, 100, 100); //c==1
		//else if (c == 2) privateZone = cv::Rect(-100, -100, 100, 100); //c==2
		//else if (c == 3) privateZone = cv::Rect(-100, 0, 100, 100); //c==3
		privateZone += cv::Point((c == 0 || c == 1) ? 0 : -1, (c == 0 || c == 3) ? 0 : -1) * 100;
		privateZone += cv::Point(frameBGR.size() / 2);
		std::cout << privateZone << std::endl;
		cv::Mat roiOuterBorder(thresholdedImages[OUTER_BORDER], privateZone);
		cv::Mat roiField(thresholdedImages[FIELD], privateZone);
		bool cb = borderCollisonEnabled ? cv::countNonZero(roiOuterBorder) > 300 : false;
		bool cu = fieldCollisonEnabled ? cv::countNonZero(roiField) < 9000 : false;
		//if(c==1) {
		//	std::cout << "coll b: " << cv::countNonZero(roiField) << std::endl;
		//}
		if (cb || cu) {
			if (!collisionWithBorder) {// no previous collison
				localState.collisionRange.x = c * 90. - 180;
				localState.collisionRange.y = c * 90. - 90;
			}
			else if (localState.collisionRange.y + 90. < c*90. - 90.) {
				localState.collisionRange.x = c * 90. - 180;
			}
			else {
				localState.collisionRange.y = c * 90. - 90;
			}
			collisionWithBorder |= cb;
			collisonWithUnknown |= cu;
			if (!hideUseless) {
				cv::rectangle(frameBGR, privateZone, cv::Scalar(cb * 64 + cu * 128, 0, 255), 2, 8);
			}

		}
		else {
			if (!hideUseless) {
				cv::rectangle(frameBGR, privateZone, cv::Scalar(155, 255, 155), 2, 8);
			}
		}
		//std::cout << "coll b: " << cv::countNonZero(roiOuterBorder) << std::endl;
	}
	localState.collisionWithBorder = collisionWithBorder;
	localState.collisionWithUnknown = collisonWithUnknown;
}
		else {
			localState.collisionWithBorder = false;
			localState.collisionWithUnknown = false;
		}

}

void MainCameraVision::Start() {
	try {
		for (int i = 0; i < NUMBER_OF_OBJECTS; i++) {
			objectThresholds[(OBJECT)i] = m_pCamera->GetObjectThresholds(i, OBJECT_LABELS[(OBJECT)i]);
		}
		ThreadedClass::Start();
	}
	catch (...){
		std::cout << "Calibration data is missing!" << std::endl;

	}
}
