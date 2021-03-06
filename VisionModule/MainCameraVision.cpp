#include "MainCameraVision.h"


#include <queue>          // std::priority_queue
#include <functional>     // std::greater
//#include "VideoRecorder.h"
#include "../CommonModule/FieldState.h"
#include "../CommonModule/RobotState.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#ifdef SHOW_UI
extern IDisplay * display;
#endif // SHOW_UI


extern FieldState gFieldState;
extern RobotState gRobotState;
extern std::map<OBJECT, std::string> OBJECT_LABELS;

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

MainCameraVision::MainCameraVision(ICamera *pCamera, const std::string sName) : ConfigurableModule(sName), ThreadedClass(sName),
thresholdObjects({ BALL, BLUE_GATE, YELLOW_GATE, FIELD, INNER_BORDER, OUTER_BORDER })
{
	m_pCamera = pCamera;

	ADD_BOOL_SETTING(gaussianBlurEnabled);
	ADD_BOOL_SETTING(greenAreaDetectionEnabled);
	ADD_BOOL_SETTING(gateObstructionDetectionEnabled);
	ADD_BOOL_SETTING(borderDetectionEnabled);
	ADD_BOOL_SETTING(borderCollisonEnabled);
	ADD_BOOL_SETTING(fieldCollisonEnabled);
	ADD_BOOL_SETTING(nightVisionEnabled);
	ADD_BOOL_SETTING(detectOtherRobots);
	ADD_BOOL_SETTING(detectObjectsNearBall);
	ADD_BOOL_SETTING(useKalmanFilter);
//	videoRecorder = new VideoRecorder("videos/", 30, m_pCamera->GetFrameSize(true));
	LoadSettings();

	if(pCamera != nullptr)
		Start();
}


MainCameraVision::~MainCameraVision()
{
	WaitForStop();
	if (thresholder == nullptr)
		delete thresholder;

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
	frameCenter = cv::Point2d(m_pCamera->GetFrameSize()) / 2;
	double t1 = (double)cv::getTickCount();
	double t0 = (double)cv::getTickCount();
	size_t counter = 0;
	double fps = 0.;

	while (!stop_thread) {

		double t2 = (double)cv::getTickCount();
		double dt = (t2 - t1) / cv::getTickFrequency();
		double dt2 = (t2 - t0) / cv::getTickFrequency();
		if (counter > 10) {
			fps = (double)counter / dt;
			t1 = t2;
			counter = 0;
		}
		counter++;

		frameBGR = m_pCamera->Capture();
#ifdef SHOW_UI
		cv::putText(frameBGR, std::to_string(fps), cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.9, cv::Scalar(23, 40, 245));
#endif // _DEBUG

		if (m_bEnabled) {

			if (gaussianBlurEnabled) {
				cv::GaussianBlur(frameBGR, frameBGR, cv::Size(3, 3), 4);
			}
			cvtColor(frameBGR, frameHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


			ProcessFrame(dt2);
			t0 = t2;
			{
				boost::mutex::scoped_lock lock(state_mutex); //allow one command at a time
				memcpy(&localStateCopy, &localState, sizeof(FieldState));
				memcpy(&localStateCopy.balls, &lastBalls, sizeof(BallPosition)*MAX_BALLS); //TODO:avoid double copy
				

				stateUpdated = true;
			}
			ResetUpdateState();
		}
		else {
			;//sleep
			Sleep(10);
		}
#ifdef SHOW_UI
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(0, -30), (frameSize / 2) + cv::Size(0, 30), cv::Scalar(0, 0, 255), 3, 8, 0);
		//cv::line(frameBGR, (frameBGR.size / 2) + cv::Size(-30, 0), (frameSize / 2) + cv::Size(30, 0), cv::Scalar(0, 0, 255), 3, 8, 0);
		display->ShowImage(ThreadedClass::name, frameBGR);
		
#endif // DEBUG

	}
}
void MainCameraVision::ResetUpdateState(){
	// reset all
	for (size_t i = 0; i < MAX_BALLS; i++) {
		newBalls[i].isValid = false;
		newBalls[i].isUpdated = false;
		newBalls[i].isPredicted = false;
		newBalls[i].distance = 10001;
		newBalls[i].heading = 0;
		newBalls[i].angle = 0;

	}
	localState.gates[BLUE_GATE].isValid = false;
	localState.gates[BLUE_GATE].distance = 10001;

	localState.gates[YELLOW_GATE].isValid = false;
	localState.gates[YELLOW_GATE].distance = 10001;

}
bool MainCameraVision::PublishState() {
	boost::mutex::scoped_lock lock(state_mutex); //allow one command at a time
	if (stateUpdated) {
		//memcpy(&gFieldState, &localStateCopy, sizeof(FieldState));
		memcpy(&gFieldState.balls, &localStateCopy.balls, MAX_BALLS * sizeof(BallPosition));
		memcpy(&gFieldState.gates, &localStateCopy.gates, 2 * sizeof(GatePosition));
		//memcpy(&gFieldState.self, &localStateCopy.self, sizeof(ObjectPosition));
		memcpy(&gFieldState.partner, &localStateCopy.self, sizeof(ObjectPosition));
		memcpy(&gFieldState.opponents, &localStateCopy.gates, 2 * sizeof(ObjectPosition));
		gFieldState.closestBall = localStateCopy.closestBall;

		stateUpdated = false;
		return true;
	}
	return false;
}
void  MainCameraVision::ProcessFrame(double dt) {

#ifdef SHOW_UI
	cv::line(frameBGR, (frameBGR.size() / 2) + cv::Size(0, -30), (frameBGR.size() / 2) + cv::Size(0, 30), cv::Scalar(0, 0, 255), 3, 8, 0);

	cv::line(frameBGR, (frameBGR.size() / 2) + cv::Size(-30, 0), (frameBGR.size() / 2) + cv::Size(30, 0), cv::Scalar(0, 0, 255), 3, 8, 0);
#endif

	ThresholdFrame();
	//CheckGateObstruction();
	FindGates();

	//CheckCollisions();
	FindBalls();
	FindMissingBalls(dt);
	FindClosestBalls();


}
void MainCameraVision::ThresholdFrame() {
	if (thresholder == nullptr) {
		for (auto &object : thresholdObjects) {
			thresholdedImages[object] = cv::Mat(frameHSV.rows, frameHSV.cols, CV_8U, cv::Scalar::all(0));
		}
		thresholder = new TBBImageThresholder(thresholdedImages, objectThresholds);
	}
	thresholder->Start(frameHSV, thresholdObjects);
#ifdef SHOW_UI
	for (auto &object : thresholdObjects) {
		display->ShowImage(ThreadedClass::name + "::" + OBJECT_LABELS[object], thresholdedImages[object]);
	}
#endif
}

void MainCameraVision::UpdateObjectPostion(ObjectPosition & object, const cv::Point2d &pos) {
	object.rawPixelCoords = pos - frameCenter;
	if (pos.x < 0) {
		object.isValid = false;
		return;
	}
	double dist = cv::norm(object.rawPixelCoords);

	double distanceInCm = dist == 0 ? 0.0 : std::max(0.0, 13.13*exp(0.008 * dist));

	double angle = angleBetween(pos - frameCenter, { -1, 0 });
	//double angle = atan((object.rawPixelCoords.y) / (object.rawPixelCoords.x)) * 180 / PI;
	//TODO: hack to fix simulator, as 
	if (distanceInCm < 14 && fabs(fabs(angle) - 270)<0.01)  angle = 0;
	// flip angle alony y axis
	object.polarMetricCoords = { distanceInCm, 360-angle};
	SYNC_OBJECT(object);
	object.isValid = true;
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

void MainCameraVision::FindGates() {
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
		UpdateObjectPostion(localState.gates[BLUE_GATE], blueGateCenter);
		
	}

	//Yellow gate pos
	bool yellowFound = yellowGateFinder.Locate(thresholdedImages[YELLOW_GATE], frameHSV, frameBGR, yellowGateCenter, yellowGate, notYellowGates);
	if (yellowFound) {
		cv::Point vertices[4];
		for (int i = 0; i < 4; ++i) {
			vertices[i] = yellowGate[i];
		}
		cv::fillConvexPoly(thresholdedImages[BALL], vertices, 4, cv::Scalar::all(0));
		UpdateObjectPostion(localState.gates[YELLOW_GATE], yellowGateCenter);

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

#ifdef SHOW_UI
			cv::circle(frameBGR, c2, 12, color2, -1, 8, 0);
			cv::circle(frameBGR, c1, 12, color4, -1, 12, 0);
#endif

		UpdateObjectPostion(localState.gates[BLUE_GATE], c1);
		UpdateObjectPostion(localState.gates[YELLOW_GATE], c2);


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
void MainCameraVision::FindBalls() {
	std::vector<cv::Point2d> balls;
	bool ballsFound = ballFinder.Locate(thresholdedImages[BALL], frameHSV, frameBGR, balls);
	localState.ballCount = 0;
	for (auto ball : balls) {
		// this is dangerous as fixed size array is used. TODO: convert balls back to vector perhaps.
		if (ballFinder.validateBall(thresholdedImages, ball, frameHSV, frameBGR)) {
			UpdateObjectPostion(newBalls[localState.ballCount], ball);
			newBalls[localState.ballCount].id = 0;
			localState.ballCount++;
		}
		if (localState.ballCount >= MAX_BALLS) break;
	}
	//if (localState.ballCount > 11) {
	//	cv::imshow("err", frameBGR);
	//	cv::waitKey(0);
	//}

}

void MainCameraVision::FindMissingBalls(double dt){
	
	cv::Rect2d r(-35, -35, 70, 70);
	
	// copy newBalls into lastBalls keeping old ones if not expired
	// first find what old balls are present
	for (auto &ball2 : lastBalls){
		if (!ball2.isValid) continue;
		if (ball2.isPredicted && ball2.lostTime > 0.5) {
			ball2.isValid = false;
			continue;
		}
		ball2.isUpdated = false;
		for (auto &ball1 : newBalls){
			if (ball1.isUpdated) continue;
			if (ball2.isValid && (r + ball2.rawPixelCoords).contains(ball1.rawPixelCoords)){
				// override last ball with new
				uchar id = ball2.id;
				memcpy(&ball2, &ball1, sizeof(BallPosition));
				ball2.isUpdated = true;
				ball2.id = id; // restore
				ball2.lostTime = 0;
				ball1.isUpdated = true;
				break;
			}
		}
		if (!ball2.isUpdated){
			ball2.isUpdated = true;
			ball2.isPredicted = true;
			ball2.lostTime += dt;
		}
	}

	//then add new ones
	int i = 0;
	for (auto &ball2 : newBalls){
		if (!ball2.isValid) continue;
		if (!ball2.isUpdated){
			for (; i < MAX_BALLS; i++){
				auto &ball1 = lastBalls[i];
				if (!ball1.isValid) {
					memcpy(&ball1, &ball2, sizeof(BallPosition));
					ball1.isUpdated = true;
					ball1.lostTime = 0;
					ball1.id = ++ballCounter;
					//std::cout << "ball lost" << ball1.rawPixelCoords << std::endl;
					break;
				}
			}
		}
	}
	/*
	if (lostBallCount > 0) {
		std::cout << "lost count: " << lostBallCount << ", couunter" << ballCounter << std::endl;
		//assert(lostBallCount < 5);
	}
	*/
#ifdef SHOW_UI
	cv::Rect2d r2(-30, -30, 60, 60);

	for (auto &ball1 : lastBalls){
		if (!ball1.isUpdated) continue;
		cv::Scalar c(0, ball1.isUpdated ? 255:0, ball1.isPredicted ? 255 : 0 );
		//cv::rectangle(frameBGR, r2 + frameCenter + ball1.rawPixelCoords, c, 3, 8, 0);
		cv::putText(frameBGR, std::to_string(ball1.id), ball1.rawPixelCoords + frameCenter + cv::Point2d(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(23, 40, 245));
	}
#endif // SHOW_UI

}

void MainCameraVision::FindClosestBalls(){
	uchar closest = MAX_BALLS, closest2 = MAX_BALLS, closest3 = MAX_BALLS-1;
	double dist1 = INT_MAX, dist2 = INT_MAX, dist3 = INT_MAX;
	localState.closestBallInFront = MAX_BALLS - 1;
	localState.closestBall = MAX_BALLS - 1;
	uchar i=0;
	for (auto &ball : lastBalls){
		if (!ball.isValid) continue;
		localState.ballCount++;

		if (ball.distance < dist1){
			localState.closestBall = i;
			dist1 = ball.distance;
		}
		i++;

		//if (ball.distance < dist2 && abs(ball.heading) < 90){
		//	localState.closestBallInFront = i;
		//	dist2 = ball.distance;
		//}
	};
#ifdef SHOW_UI
	cv::Scalar redColor(255, 0, 255);
	cv::Scalar greenColor(0, 255, 255);

	cv::Rect privateZone(-19, -19, 38, 38);
	cv::Rect privateZone2(-15, -15, 30, 30);
	cv::Point p1 = lastBalls[localState.closestBall].rawPixelCoords + frameCenter;
	//cv::Point p2 = newBalls[localState.closestBallInFront].rawPixelCoords + frameCenter;
	if (lastBalls[localState.closestBall].isPredicted) {
		rectangle(frameBGR, privateZone.tl() + p1, privateZone.br() + p1, greenColor, 3, 8, 0);
	}
	else {
		rectangle(frameBGR, privateZone2.tl() + p1, privateZone2.br() + p1, redColor, 3, 8, 0);
	}
#endif
}
void MainCameraVision::FindOtherRobots() {
	// TODO: this will need to be changed
	#pragma message("TODO: Reimplement MainCameraVision::FindOtherRobots")
	if (detectOtherRobots) {

		std::vector<cv::Point2i> robots;
		cv::bitwise_or(thresholdedImages[OUTER_BORDER], thresholdedImages[FIELD], thresholdedImages[FIELD]);
		bool robotsFound = robotFinder.Locate(thresholdedImages[FIELD], frameHSV, frameBGR, robots);
#ifdef SHOW_UI
			for (auto robot : robots) {
				cv::Rect robotRectangle = cv::Rect(robot - cv::Point(20, 20) + cv::Point(frameBGR.size() / 2),
					robot + cv::Point(20, 20) + cv::Point(frameBGR.size() / 2));
				rectangle(frameBGR, robotRectangle.tl(), robotRectangle.br(), cv::Scalar(10, 255, 101), 2, 8, 0);
			}
#endif
		bool ourRobotBlueBottom = (gRobotState.ourTeam == TEAM_PINK);
		//std::vector<cv::Point2d> robots;
		//bool ballsFound = ballFinder.Locate(thresholdedImages[FIELD], frameHSV, frameBGR, robots);

		std::vector<std::pair<cv::Point2i, double>> positionsToDistances; //One of the colors position and according distances
		for (size_t blueIndex = 0; blueIndex < notBlueGates.size(); blueIndex++) {
			for (size_t yellowIndex = 0; yellowIndex < notYellowGates.size(); yellowIndex++) {
				cv::Point2d bluePos = notBlueGates[blueIndex];
				cv::Point2d yellowPos = notYellowGates[yellowIndex];
				double distBetweenYellowBlue = cv::norm(bluePos - yellowPos);
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
			UpdateObjectPostion(localState.partner, positionsToDistances[0].first);
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
#ifdef SHOW_UI
				cv::rectangle(frameBGR, privateZone, cv::Scalar(cb * 64 + cu * 128, 0, 255), 2, 8);
#endif
		}
		else {
#ifdef SHOW_UI
				cv::rectangle(frameBGR, privateZone, cv::Scalar(155, 255, 155), 2, 8);
#endif
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
