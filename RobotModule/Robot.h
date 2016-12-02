#include "../CommonModule/Types.h"
#include "../CommonModule/RobotState.h"
#include "../CommonModule/Interfaces.h"
#include <atomic>
#include <boost/thread/mutex.hpp>
#include "../CommonModule/UdpServer.h"
#include <boost/asio.hpp>

class Robot: public UdpServer {
private:
	IVisionModule *m_pMainVision = NULL;
	IVisionModule *m_pFrontVision = NULL;
	ISoccerRobot *m_pComModule = NULL;
	std::map<RunMode, IStateMachine *> m_AutoPilots;
//	RunMode curPlayMode = ROBOT_MODE_IDLE;
//	RunMode lastPlayMode = ROBOT_MODE_IDLE;
	bool master;

	void Run();
    boost::mutex remote_mutex;
	bool debug = false;
	bool debug_step = false;
	uchar lastGameMode = GAME_MODE_END_HALF;

protected:
	OBJECT targetGate= NUMBER_OF_OBJECTS; //uselected
	bool captureFrames = false;
	std::atomic_bool autoPilotEnabled;
	std::string play_mode = "single";
//	SimpleSerial *serialPort;
	boost::asio::io_service &io;
public:
    Robot(boost::asio::io_service &io, ICamera *pMainCamera, ICamera *pFrontCamera, ISoccerRobot*, bool master);
	bool Launch();
	~Robot();

	void SendFieldState();
	bool MessageReceived(const boost::array<char, BUF_SIZE>& buffer, size_t size);
	bool MessageReceived(const std::string & message);
	std::string lastMessage = "";
	void InitializeTarget();
};
