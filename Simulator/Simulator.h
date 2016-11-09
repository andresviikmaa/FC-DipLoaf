#pragma once
#include "../CommonModule/Types.h"
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/Interfaces.h"
#include "../CommonModule/UdpServer.h"

#include <mutex>

const int MAX_ROBOTS = 10;
class Simulator : public ICamera, public ISerial, public ThreadedClass, public UdpServer/*, public RefereeCom*/
{
	class FrontCamera : public ICamera {
	protected:
		std::string sName = "front";
		Simulator * pSim;
		cv::Mat front_frame;

	public:
		FrontCamera(Simulator* sim) {
			pSim = sim;
		}
		cv::Mat & Capture(bool bFullFrame = false);
		cv::Size GetFrameSize(bool bFullFrame = false) {
			return pSim->front_frame.size();
		}
		double GetFPS() { 
			return pSim->GetFPS();
		};
		cv::Mat & GetLastFrame(bool bFullFrame = false) {
			return pSim->front_frame;
		}
		void TogglePlay() {};
		HSVColorRange GetObjectThresholds(int index, const std::string &name) {
			return pSim->GetObjectThresholds(index, name);
		}
		const std::string & getName() { return sName;  }
		double getDistanceInverted(const cv::Point2d &pos, const cv::Point2d &orgin) const;

		double Hfov = 35.21;
		double Vfov = 21.65; //half of cameras vertical field of view (degrees)
		double CamHeight = 345; //cameras height from ground (mm)
		double CamAngleDev = 26; //deviation from 90* between ground
	};
	using UdpServer::SendMessage;
public:
	Simulator(boost::asio::io_service &io, bool master, const std::string game_mode);
	virtual ~Simulator();
	ICamera & GetFrontCamera() {
		return m_frontCamera;
	}
	std::string sName = "main";
	const std::string & getName() { return sName; }
	virtual cv::Mat & Capture(bool bFullFrame = false);
	virtual cv::Size GetFrameSize(bool bFullFrame = false);
	virtual double GetFPS();
	virtual cv::Mat & GetLastFrame(bool bFullFrame = false);
	virtual void TogglePlay();
	virtual void Drive(double fowardSpeed, double direction = 0, double angularSpeed = 0);
	virtual const Speed & GetActualSpeed();
	virtual const Speed & GetTargetSpeed();
	virtual void Init();
	std::string GetDebugInfo();
	bool IsReal() { return false; }
	void Run();
	virtual void SetTargetGate(OBJECT gate) {}
	virtual GatePosition &GetTargetGate() { return blueGate; };
	virtual GatePosition &GetHomeGate() { return yellowGate; };
	virtual bool BallInTribbler();
	virtual void Kick(int force = 800);
	virtual void ToggleTribbler(bool start) {
		tribblerRunning = start;
	};

	virtual void DataReceived(const std::string & message);//serial
	virtual bool MessageReceived(const std::string & message); // UDP

	void giveCommand(GameMode command);
	void SendCommand(int id, const std::string &cmd, int param = INT_MAX) {
		std::ostringstream oss;

		oss << id << ":" << cmd;
		if (param < INT_MAX) oss << param;
		oss << "\n";
		WriteString(oss.str());
	};

	virtual void WriteString(const std::string &s);
	//virtual void DataReceived(const std::string & message){};
	virtual void SetMessageHandler(ISerialListener *callback) {
		messageCallback = callback;
	};
	virtual void SendPartnerMessage(const std::string message) {};
	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name);
	double getDistanceInverted(const cv::Point2d &pos, const cv::Point2d &orgin) const;
	virtual void PollData() {
		;
	}
protected:
	ISerialListener *messageCallback = NULL;
	FrontCamera m_frontCamera;
	double orientation;
	// main camera
	cv::Mat frame = cv::Mat(1024, 1280, CV_8UC3);
	cv::Mat frame_blank = cv::Mat(1024, 1280, CV_8UC3, cv::Scalar(21, 188, 80));
	// front camera
	cv::Mat front_frame = cv::Mat(480, 640, CV_8UC3);
	cv::Mat front_frame_copy = cv::Mat(480, 640, CV_8UC3);;
	cv::Mat front_frame_blank = cv::Mat(480, 640, CV_8UC3, cv::Scalar(21, 188, 80));

	cv::Point2d cameraOrgin = cv::Point2d(512, 640);
	Speed targetSpeed, actualSpeed;
	void UpdateGatePos();
	void UpdateBallPos(double dt);
	void UpdateRobotPos(double dt);
	void UpdateBallIntTribbler(cv::Mat robotSpeed, double dt);
	std::mutex mutex;
	ObjectPosition robots[MAX_ROBOTS];
	std::map<uchar, cv::Scalar>  colors;

	ushort ballCount;
	void drawRect(cv::Rect rec, int thickness, const cv::Scalar &color);
	void drawLine(cv::Point start, cv::Point end, int thickness, CvScalar color);
	void drawCircle(cv::Point start, int radius, int thickness, CvScalar color);

	cv::Point Simulator::MainCamPos(cv::Point2d pos);
	cv::Point Simulator::FrontCamPos(cv::Point2d pos);
private:
	int mNumberOfBalls;
	int frames = 0;
	double fps;
	bool tribblerRunning = false;
	double time = (double)cv::getTickCount();
	double time2 = (double)cv::getTickCount();
	double state_time = (double)cv::getTickCount();
	bool isMaster = false;
	bool isMasterPresent = false;
	int id = -1;
	int next_id = 1;
	bool stop_send = false;
	bool ball_in_tribbler = false;
	cv::Mat wheelSpeeds = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 0.0);

	BallPosition balls[12];
	GatePosition blueGate;
	GatePosition yellowGate;
	RobotPosition self; //Robot distance on field
};
