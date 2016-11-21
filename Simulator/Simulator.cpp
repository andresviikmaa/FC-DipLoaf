// Simulator.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#include <chrono>
#include <thread>
#include <time.h>       /* time */
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include "Simulator.h"
#include "../RobotModule/Robot.h"
#include "../DisplayModule/Dialog.h"
#include "../VisionModule/MainCameraVision.h"
#include "../StateMachine/SingleModePlay.h"
#include "../StateMachine/MultiModePlay.h"
#include "../HardwareModule/ComModule.h"
#include "opencv2/imgproc.hpp"
#include <boost/exception/diagnostic_information.hpp> 
#include <boost/exception_ptr.hpp>

namespace po = boost::program_options;

extern cv::Mat wheelAngles;
double angleBetween(const cv::Point2d &a, const cv::Point2d &b);

const double SIMULATOR_SPEED = 0.5;
const bool INIT_RANDOM = false;

Simulator::Simulator(boost::asio::io_service &io, bool master, const std::string game_mode) :
	mNumberOfBalls(game_mode == "master" || game_mode == "slave" ? 1 : 19)
	, ThreadedClass("Simulator"), UdpServer(io, 31000, master)
	, isMaster(master)
	, ballCount(game_mode == "master" || game_mode == "slave" ? 1 : 19),
	m_frontCamera(this), mainboard(io, this)
{
	blueGate.fieldCoords = cv::Point(0, 230);	
	yellowGate.fieldCoords = cv::Point(0, -230);

	srand((unsigned int) ::time(NULL));
	/*
	wheelSpeeds.push_back({ 0, 0 });
	wheelSpeeds.push_back({ 0, 0 });
	wheelSpeeds.push_back({ 0, 0 });
	wheelSpeeds.push_back({ 0, 0 });
	*/
	self.fieldCoords = !INIT_RANDOM ? cv::Point(155, 230) : cv::Point(rand() % 300 - 150, rand() % 460 - 230);
	self.polarMetricCoords = cv::Point(0, !INIT_RANDOM ? -30 : rand() % 359);
	self.fieldCoords = cv::Point(0, 230);
	self.polarMetricCoords.y = 0;
	colors.insert(std::make_pair(BLUE_GATE, cv::Scalar(236, 137, 48)));
	colors.insert(std::make_pair(BALL, cv::Scalar(48, 154, 236)));
	colors.insert(std::make_pair(YELLOW_GATE, cv::Scalar(61, 255, 244)));
	colors.insert(std::make_pair(FIELD, cv::Scalar(21, 188, 80)));
	colors.insert(std::make_pair(INNER_BORDER, cv::Scalar(255,255,255)));
	colors.insert(std::make_pair(OUTER_BORDER, cv::Scalar(0,0,0)));

	if (isMaster) {
		id = 0;
		// distribute balls uniformly at random
		if (mNumberOfBalls == 1) {
			balls[0].fieldCoords = { 0, 0 };
			balls[0].id = 0;
			if (game_mode == "master") self.fieldCoords = { 0, 60 };
			robots[1].fieldCoords = cv::Point(rand() % 300 - 150, rand() % 460 - 230);

		}
		else {
			for (int i = 0; i < mNumberOfBalls; i++) {
				balls[i].fieldCoords.x = (int)(((i % 3) - 1) * 100) + (!INIT_RANDOM ? 0 : (rand() % 200) - 100);
				balls[i].fieldCoords.y = (int)((i / 3 - 2.5) * 40) + (!INIT_RANDOM ? 0 : (rand() % 200) - 100);
				balls[i].id = i;
			}
			robots[9].fieldCoords = cv::Point(rand() % 300 - 150, rand() % 460 - 230);
		}
	}
	else {
		SendMessage("ID? #"); // ask slave id
	};
	Start();
}
void Simulator::HandleMainBoardCommand(const std::string &command) {

	std::vector<std::string> tokens;
	boost::split(tokens, command, boost::is_any_of(":"));
	std::string cmd = tokens[1];
	if (cmd == "speeds" && tokens.size() > 5) {
		wheelSpeeds.at<double>(0, 0) = atoi(tokens[1].c_str());
		wheelSpeeds.at<double>(1, 0) = atoi(tokens[2].c_str());
		wheelSpeeds.at<double>(2, 0) = atoi(tokens[3].c_str());
		wheelSpeeds.at<double>(3, 0) = atoi(tokens[4].c_str());
		ToggleTribbler(atoi(tokens[5].c_str()) > 0);
	} else if (cmd =="kick"){
		Kick(atoi(tokens[1].c_str()));
	}

}

bool Simulator::MessageReceived(const std::string & message) { //udp
	std::stringstream ss(message);
	std::string command, r_id;
	ss >> command;
	if (isMaster) {
		if (command == "ID?") {
			stop_send = true;
			SendMessage("ID= " + std::to_string(next_id++) + " #");
		}
		else if (command == "POS") { // foward to slaves
			SendMessage(message);
		}
		else if (command == "ACK") { // id received
			stop_send = false;
		}
		else if (command == "KCK") {
			double s, a;
			ss >> r_id >> s >> a;
			int _id = atoi(r_id.c_str());
			balls[_id].speed = s;
			balls[_id].heading = a;
		}
	}
	else { // slave commands
		if (command == "ID=") {
			ss >> r_id;
			id = atoi(r_id.c_str());
			SendMessage("ACK #");
		}
		else if (command == "BAL") {
			ss >> r_id;
			int _id = atoi(r_id.c_str());
			if (_id != id) {
				std::string x, y, a;
				ss >> x >> y;
				balls[_id].fieldCoords.x = atoi(x.c_str());
				balls[_id].fieldCoords.y = atoi(y.c_str());
			}
		}
		else if (command == "STT") {
			int numballs;
			ss >> numballs;
			for (int i = 0; i < mNumberOfBalls; i++) {
				std::string x, y, a;
				ss >> x >> y;
				balls[i].fieldCoords.x = atoi(x.c_str());
				balls[i].fieldCoords.y = atoi(y.c_str());
			}
			std::string r_id;
			do {
				std::string x, y, a;
				ss >> r_id >> x >> y;
				if (r_id != "99") {
					int _id = atoi(r_id.c_str());
					if (_id != id) {
						robots[_id].fieldCoords.x = atoi(x.c_str());
						robots[_id].fieldCoords.y = atoi(y.c_str());
					}
				}

			} while (r_id != "99");
		}
		else if (command == "REF") {
			int ref_command;
			ss >> ref_command;
			assert(false); //fixme RefereeCom::giveCommand((FieldState::GameMode)ref_command);
		}

	}
	if (command == "POS") {
		ss >> r_id;
		int _id = atoi(r_id.c_str());
		if (_id != id) {
			std::string x, y, a;
			ss >> x >> y >> a;
			robots[_id].fieldCoords.x = atoi(x.c_str());
			robots[_id].fieldCoords.y = atoi(y.c_str());
			robots[_id].angle = atoi(a.c_str());
			assert(false); //heading ??
		}
	}
	if (id < 0) {
		SendMessage("ID? #"); // ask slave id again
	}
	return true;
}
void Simulator::UpdateGatePos() {

	frame_blank.copyTo(frame);
	front_frame_blank.copyTo(front_frame);

	drawRect(cv::Rect(cv::Point(-155, -230), cv::Point(155, 230)), 10, colors[OUTER_BORDER]);
	drawRect(cv::Rect(cv::Point(-145, -220), cv::Point(145, 220)), 10, colors[INNER_BORDER]);
	drawLine(cv::Point(-145, 0), cv::Point(145, 0), 10, colors[INNER_BORDER]);
	drawCircle(cv::Point(0, 0), 40, 10, colors[INNER_BORDER]);

	blueGate.polarMetricCoords.x = cv::norm(self.fieldCoords - blueGate.fieldCoords);
	blueGate.polarMetricCoords.y = 360 - angleBetween(cv::Point(0, 1), self.fieldCoords - (blueGate.fieldCoords)) + self.angle;
	yellowGate.polarMetricCoords.x = cv::norm(self.fieldCoords - yellowGate.fieldCoords);;
	yellowGate.polarMetricCoords.y = 360 - angleBetween(cv::Point(0, 1), self.fieldCoords - (yellowGate.fieldCoords)) + self.angle;
	SYNC_OBJECT(blueGate);
	SYNC_OBJECT(yellowGate);

	for (int s = -1; s < 2; s += 2) {
		cv::Point2d shift1(s * 10, -20);
		cv::Point2d shift2(s * 10, 20);



		double g1 = cv::norm(self.fieldCoords - blueGate.fieldCoords);
		double g2 = cv::norm(self.fieldCoords - yellowGate.fieldCoords);
		double s1 = g1 > 0 ? 8000 / g1 : 90;
		double s2 = g2 > 0 ? 8000 / g2 : 90;
		double fs1 = g1 > 0 ? 8000 / g1 : 30;
		double fs2 = g2 > 0 ? 8000 / g2 : 30;

		cv::circle(frame, MainCamPos(yellowGate.fieldCoords + shift1), s2, colors[YELLOW_GATE], -1);
		cv::circle(frame, MainCamPos(blueGate.fieldCoords + shift2), s1, colors[BLUE_GATE], -1);

		// front camera
		cv::circle(front_frame, FrontCamPos(yellowGate.fieldCoords + shift1), fs2, colors[YELLOW_GATE], -1);
		cv::circle(front_frame, FrontCamPos(blueGate.fieldCoords + shift1), fs1, colors[BLUE_GATE], -1);

	}

}
void Simulator::UpdateBallPos(double dt) {
	std::stringstream message;
	message << "STT " << mNumberOfBalls << " ";
	// balls 
	for (int i = 0; i < mNumberOfBalls; i++) {
		if (isMaster) {
			if (balls[i].speed > 0.001) {
				balls[i].fieldCoords.x += balls[i].speed*dt * (sin(balls[i].heading / 180 * CV_PI));
				balls[i].fieldCoords.y -= balls[i].speed*dt * (cos(balls[i].heading / 180 * CV_PI));
				balls[i].speed *= 0.8;
			}
			message << (int)balls[i].fieldCoords.x << " " << (int)balls[i].fieldCoords.y << " ";
			//SendMessage(message.str());
		}
		double a = angleBetween(cv::Point(0, -1), self.fieldCoords - balls[i].fieldCoords) + self.angle;
		double d = getDistanceInverted(self.fieldCoords, balls[i].fieldCoords);
		double x = -d*sin(a / 180 * CV_PI);
		double y = d*cos(a / 180 * CV_PI);
		balls[i].polarMetricCoords.x = cv::norm(self.fieldCoords - balls[i].fieldCoords);
		a -= 180;
		if (a > 360) a -= 360;
		if (a < 0) a += 360;
		balls[i].polarMetricCoords.y = a;
		SYNC_OBJECT(balls[i]);
		int r1 = 900000./pow(d,2);
		int r2 = 900000./pow(d,2);
		cv::circle(frame, MainCamPos(balls[i].fieldCoords), r1, colors[BALL] , -1);
		// front camera
		cv::circle(front_frame, FrontCamPos(balls[i].fieldCoords), r2, colors[BALL], -1);

	}
	if (isMaster) {
		message << 0 << " " << self.fieldCoords.x << " " << self.fieldCoords.y << " ";
	}
	// draw shared robots
	for (int i = 0; i < MAX_ROBOTS; i++) {
		if (abs(robots[i].fieldCoords.x) > 1000) continue;
		double a = angleBetween(cv::Point(0, -1), self.fieldCoords - robots[i].fieldCoords) + self.angle;
		double d = getDistanceInverted(self.fieldCoords, robots[i].fieldCoords);
		double x = -d*sin(a / 180 * CV_PI);
		double y = d*cos(a / 180 * CV_PI);
		cv::Scalar color(i * 52, i * 15 + 100, i * 30);
		double s = std::min(20., 4000 / cv::norm(self.fieldCoords - robots[i].fieldCoords));
		cv::circle(frame, cv::Point(x, y) + cv::Point(frame.size() / 2), s * 2, color, -1);
		cv::Scalar color1(236, 137, 48);
		cv::Scalar color2(61, 255, 244);
		cv::rectangle(frame, cv::Point2d(x - s, y - s) + cv::Point2d(frame.size() / 2), cv::Point2d(x + s, y) + cv::Point2d(frame.size() / 2), color1, -1);
		cv::rectangle(frame, cv::Point2d(x - s, y) + cv::Point2d(frame.size() / 2), cv::Point2d(x + s, y + s) + cv::Point2d(frame.size() / 2), color2, -1);
		if (isMaster) {
			message << i << " " << (int)robots[i].fieldCoords.x << " " << (int)robots[i].fieldCoords.y << " ";
		}
	}
	double t2 = (double)cv::getTickCount();
	double dt2 = (t2 - state_time) / cv::getTickFrequency();

	if (isMaster && dt2 > 0.1) {
		state_time = t2;
		message << " 99 0 0 #";
		SendMessage(message.str());
	}

}

void Simulator::UpdateRobotPos(double dt) {

	if (dt > 1000) return;
	cv::Mat robotSpeed = cv::Mat_<double>(3, 1);
	cv::solve(wheelAngles, wheelSpeeds, robotSpeed, cv::DECOMP_SVD);
	double dr = SIMULATOR_SPEED * 2 * (robotSpeed.at<double>(2)*dt);
	self.polarMetricCoords.y -= dr;
	if (self.polarMetricCoords.y > 360) self.polarMetricCoords.y -= 360;
	if (self.polarMetricCoords.y < -360) self.polarMetricCoords.y += 360;
	cv::Mat rotMat = getRotationMatrix2D(cv::Point(0, 0), self.angle, 1);
	cv::Mat rotatedSpeed = rotMat * robotSpeed;
	double dx = SIMULATOR_SPEED*rotatedSpeed.at<double>(0)*dt;
	double dy = SIMULATOR_SPEED*rotatedSpeed.at<double>(1)*dt;

	self.fieldCoords.x += dx;
	self.fieldCoords.y -= dy;
	SYNC_OBJECT(self);

	if (!isMaster && id > 0) {
		std::stringstream message;
		message << "POS " << id << " " << self.fieldCoords.x << " " << self.fieldCoords.y << " " << self.angle << " #";
		SendMessage(message.str());
	}

	UpdateGatePos();

	UpdateBallIntTribbler(robotSpeed, dt);
	UpdateBallPos(dt);
	cv::circle(frame, cv::Point(frame.size() / 2), 55, cv::Scalar::all(160), -1);


}

void Simulator::UpdateBallIntTribbler(cv::Mat robotSpeed, double dt) {
	bool was_in_tribbler = ball_in_tribbler;
	//---

	if (!tribblerRunning) {
		ball_in_tribbler = false;
	}
	double minDist = INT_MAX;
	double dist = INT_MAX;
	int minIndex = -1;
	for (int i = 0; i < mNumberOfBalls; i++) {
		dist = cv::norm(self.fieldCoords - balls[i].fieldCoords);
		//std::cout << dist << std::endl;
		if (dist < minDist /*&& (fabs(balls[i].heading) < 10 || was_in_tribbler || (fabs(balls[i].heading) - 90)< 1)*/) {
			minDist = dist;
			minIndex = i;
		}
	}
	if (minIndex < 0) {
		ball_in_tribbler = false;
		return;
	}
	if (minDist < (was_in_tribbler ? 30 : 24)) {
		ball_in_tribbler = fabs(balls[minIndex].heading) < 10;

		double dr = SIMULATOR_SPEED*(robotSpeed.at<double>(2)*dt);

		cv::Mat rotMat2 = getRotationMatrix2D(self.fieldCoords, dr, 1);
		cv::Mat ballPos = cv::Mat_<double>(3, 1);
		ballPos.at<double>(0) = balls[minIndex].fieldCoords.x;
		ballPos.at<double>(1) = balls[minIndex].fieldCoords.y;
		ballPos.at<double>(2) = 1;
		cv::Mat rotatedPos = rotMat2 * ballPos;
		balls[minIndex].fieldCoords.x = rotatedPos.at<double>(0);
		balls[minIndex].fieldCoords.y = rotatedPos.at<double>(1);

		cv::Mat rotMat = getRotationMatrix2D(cv::Point(0, 0), self.angle, 1);
		cv::Mat rotatedSpeed = rotMat * robotSpeed;
		double dx = SIMULATOR_SPEED*rotatedSpeed.at<double>(0)*dt;
		double dy = SIMULATOR_SPEED*rotatedSpeed.at<double>(1)*dt;

		balls[minIndex].fieldCoords.x += dx;
		balls[minIndex].fieldCoords.y -= dy;



	}
	else ball_in_tribbler = false;
	//---

	if (!was_in_tribbler && ball_in_tribbler) {
		mainboard.SendMessage("<ball:1>");
	}
	else if (was_in_tribbler && !ball_in_tribbler) {
		mainboard.SendMessage("<ball:0>");
	}
}
Simulator::~Simulator()
{
	WaitForStop();
}

cv::Mat & Simulator::Capture(bool bFullFrame) {
	double t2 = (double)cv::getTickCount();
	if (frames > 20) {
		double dt = (t2 - time) / cv::getTickFrequency();
		fps = frames / dt;
		time = t2;
		frames = 0;
	}
	else {
		frames++;
	}
	double dt = (t2 - time2) / cv::getTickFrequency();
	UpdateRobotPos(dt);
	time2 = t2;
	{
		std::lock_guard<std::mutex> lock(mutex);
		front_frame.copyTo(front_frame_copy);
	}
	return frame;

}

cv::Size Simulator::GetFrameSize(bool bFullFrame) {
	return frame.size();
}


double Simulator::GetFPS() {
	return fps;
}


cv::Mat & Simulator::GetLastFrame(bool bFullFrame) {
	return frame;
}


void Simulator::TogglePlay() {
}



void Simulator::Drive(double fowardSpeed, double direction, double angularSpeed) {
	if (mNumberOfBalls == 0)
		return;
	targetSpeed = { fowardSpeed, direction, angularSpeed };
	//std::cout << fowardSpeed  << "\t" <<  direction << "\t" << angularSpeed << std::endl;
	/*
	self.polarMetricCoords.y += angularSpeed;
	if (self.polarMetricCoords.y > 360) self.polarMetricCoords.y -= 360;
	if (self.polarMetricCoords.y < -360) self.polarMetricCoords.y += 360;
	self.fieldCoords.x += (int)(fowardSpeed * sin((direction - self.angle) / 180 * CV_PI));
	self.fieldCoords.y += (int)(fowardSpeed * cos((direction - self.angle) / 180 * CV_PI));
	*/
}


const Speed & Simulator::GetActualSpeed() {
	return actualSpeed;
}


const Speed & Simulator::GetTargetSpeed() {
	return targetSpeed;
}


void Simulator::Init() {
}

std::string Simulator::GetDebugInfo() {
	return "simulating wheels";
}

void Simulator::Run() {
	while (!stop_thread) {
		//UpdateRobotPos();
		Sleep(50);
	}
}

bool Simulator::BallInTribbler() {

	return ball_in_tribbler;
}

void Simulator::Kick(int force) {
	if (force == 0) force = 2500;
	force /= 6;
	double minDist = INT_MAX;
	double dist = INT_MAX;
	int minDistIndex = mNumberOfBalls - 1;
	for (int i = 0; i < mNumberOfBalls; i++) {
		dist = cv::norm(self.fieldCoords - balls[i].fieldCoords);
		if (dist < minDist) {
			minDist = dist;
			minDistIndex = i;
		}
	}
	if (minDistIndex < 0) {
		return;
	}
	if (isMaster) {
		balls[minDistIndex].speed = force;
		balls[minDistIndex].heading = self.angle;

	}
	else {
		SendMessage("KCK " + std::to_string(minDistIndex) + " " + std::to_string(force) + " " + std::to_string(self.angle) + " #");
	}
	//balls[minDistIndex] = balls[mNumberOfBalls - 1];
	//balls[mNumberOfBalls - 1].~BallPosition();
	//mNumberOfBalls--;
}

void Simulator::drawRect(cv::Rect rec, int thickness, const cv::Scalar &color) {
	drawLine(rec.tl(), rec.tl() + cv::Point(rec.width, 0), thickness, color);
	drawLine(rec.tl() + cv::Point(rec.width, 0), rec.br(), thickness, color);
	drawLine(rec.br() - cv::Point(rec.width, 0), rec.br(), thickness, color);
	drawLine(rec.tl(), rec.tl() + cv::Point(0, rec.height), thickness, color);

}

void Simulator::drawLine(cv::Point start, cv::Point end, int thickness, CvScalar color)
{
	const int SCALE = 8;
	cv::Mat dummyField = cv::Mat(cv::Point(608, 608) / SCALE, CV_8UC3, cv::Scalar::all(245));
	cv::LineIterator it(dummyField, start / SCALE + cv::Point(dummyField.size() / 2), end / SCALE + cv::Point(dummyField.size() / 2), 8);
	cv::Point last = { INT_MAX, INT_MAX };
	cv::Point flast = { INT_MAX, INT_MAX };

	for (int i = 0; i < it.count; i++, ++it) {
		cv::Point xy = (it.pos() - cv::Point(dummyField.size() / 2))*SCALE;
		double d1 = cv::norm(self.fieldCoords - cv::Point2d(xy));
		cv::Point cur = MainCamPos(xy);
		if (last.x < 1000) {
			cv::line(frame, last, cur, color, std::min(30.0, 4 * 1 / d1 * 600));
		}
		last = cur;


		
		cv::Point fcur = FrontCamPos(cv::Point2d(xy));
		
		if (flast.x < 1000) {
			cv::line(front_frame, flast, fcur, color, std::min(30.0, 4 * 1 / d1 * 600));
		}
		flast = fcur;
	}
	return;
}
cv::Point Simulator::MainCamPos(cv::Point2d pos) {
	double a1 = angleBetween(cv::Point(0, -1), self.fieldCoords - pos) + self.angle;
	double d1 = getDistanceInverted(self.fieldCoords, pos);

	double x1 = -d1*sin(a1 / 180 * CV_PI);
	double y1 = d1*cos(a1 / 180 * CV_PI);
	return cv::Point((int)(x1), (int)(y1)) + cv::Point(frame.size() / 2);
}
cv::Point Simulator::FrontCamPos(cv::Point2d pos) {

	double a1 = angleBetween(cv::Point(0, -1), self.fieldCoords - pos) + self.angle;



	cv::Point center = front_frame.size() / 2;
	double diag = cv::norm(self.fieldCoords - pos);
	double distanceX = sin(a1*PI / 180) * diag;
	double distanceY = cos(a1*PI / 180) * diag;
	//double distance = CamHeight / tan(angle * PI / 180);
	double angle = atan(m_frontCamera.CamHeight / distanceY) / PI * 180;
	// double angle = (Vfov * (point.y - center.y) / center.y) + CamAngleDev;
	double y = (angle - m_frontCamera.CamAngleDev) / m_frontCamera.Vfov * (-distanceY)/1.5;
	if (angle > 180) angle = angle - 360;
	double hor_space = atan(m_frontCamera.Hfov/180*PI)*distanceX;
	//double Hor_angle = atan(HorizontalDev / distance) * 180 / PI;
	double HorizontalDev = sin(angle*PI / 180) * distanceX;
	//	double HorizontalDev = (hor_space * (point.x - center.x) / center.x);
	double x = HorizontalDev *6;

	return cv::Point(x, y+ front_frame.size().height)/2 + center;

}
void Simulator::drawCircle(cv::Point start, int radius, int thickness, CvScalar color) {

	double i, angle, x1, y1;
	cv::Point last;
	for (i = 0; i < 360; i += 10)
	{
		angle = i;
		x1 = radius * cos(angle * PI / 180);
		y1 = radius * sin(angle * PI / 180);
		cv::Point cur = cv::Point(int(x1), int(y1));
		if (i > 0) {
			drawLine(last, cur, thickness, color);
		}
		last = cur;

	}
}



HSVColorRange Simulator::GetObjectThresholds(int index, const std::string &name) {

	cv::Mat rgb(1, 1, CV_8UC3, colors[index]);
	cv::Mat hsv;
	cvtColor(rgb, hsv, CV_BGR2HSV);

	return{ { hsv.data[0]-5, hsv.data[0] + 5 },{ hsv.data[1] - 5, hsv.data[1] + 5 },{ hsv.data[2] - 5, hsv.data[2] +5 } };

}

double Simulator::FrontCamera::getDistanceInverted(const cv::Point2d &pos, const cv::Point2d &orgin) const {
	return 100;
}

cv::Mat & Simulator::FrontCamera::Capture(bool bFullFrame) {
	{
		std::lock_guard<std::mutex> lock(pSim->mutex);
		pSim->front_frame_copy.copyTo(front_frame);
	}
	return front_frame;
}


po::options_description desc("Allowed options");

double Simulator::getDistanceInverted(const cv::Point2d &pos, const cv::Point2d &orgin) const {
	double dist_cm = cv::norm(pos - orgin);
	double dist_px = 125 * log(dist_cm / 13.13);
	return dist_px < 0 ? 0 : dist_px;

}


int main(int argc, char* argv[])
{
	desc.add_options()
		("help", "produce help message")
		("camera", po::value<std::string>(), "set m_pCamera index or path")
		("app-size", po::value<std::string>(), "main window size: width x height")
		("locate_cursor", "find cursor instead of ball")
		("skip-ports", "skip ALL COM port checks")
		("skip-missing-ports", "skip missing COM ports")
		("save-frames", "Save captured frames to disc")
		("simulator-mode", po::value<std::string>(), "Play mode: single, opponent, master, slave")
		("play-mode", po::value<std::string>(), "Play mode: single1, single2, opponent, master, slave")
		("twitter-port", po::value<int>(), "UDP port for communication between robots");

	std::string play_mode = "single";
	std::string simulator_mode = "master"; 

	po::variables_map config;

	po::store(po::parse_command_line(argc, argv, desc), config);
	po::notify(config);

	if (config.count("help")) {
		std::cout << desc << std::endl;
		return false;
	}
	if (config.count("play-mode"))
		play_mode = config["play-mode"].as<std::string>();
	if (config.count("simulator-mode"))
		simulator_mode = config["simulator-mode"].as<std::string>();

	cv::Size winSize(1024, 768);
	if (config.count("app-size")) {
		std::vector<std::string> tokens;
		boost::split(tokens, config["app-size"].as<std::string>(), boost::is_any_of("x"));
		winSize.width = atoi(tokens[0].c_str());
		winSize.height = atoi(tokens[1].c_str());

	}


	
	std::atomic_bool stop_io;
	stop_io = false;
	boost::asio::io_service io;
	boost::asio::io_service io2;

	std::thread io_thread([&]() {
		while (!stop_io)
		{
			io.reset();
			io.run();
		}
		std::cout << "io stopting" << std::endl;
	});

	try {
		Simulator Sim(io, simulator_mode == "master", play_mode);
		ComModule com(io, "127.0.0.1", 50022, 50021);
		Robot robot(io2, &Sim, &Sim.GetFrontCamera(), &com, play_mode == "single1");
	
		robot.Launch();
	
	}
	catch (const boost::exception & e)
	{
		std::cout << boost::diagnostic_information(e) << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << "ups, " << e.what() << std::endl;
	}
	catch (const std::string &e)
	{
		std::cout << "ups, " << e << std::endl;
	}
	catch (...)
	{
		std::cout << "ups, did not see that coming." << std::endl;
	}

	stop_io = true;
	io2.stop();
	io_thread.join();
    return 0;
}


