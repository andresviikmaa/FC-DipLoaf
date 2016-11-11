#include "../CommonModule/Types.h"
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
	std::map<std::string, IStateMachine *> m_AutoPilots;
	std::string curPlayMode = "idle";
	std::string lastPlayMode = "idle";


	void Run();
    boost::mutex remote_mutex;
protected:
	OBJECT targetGate= NUMBER_OF_OBJECTS; //uselected
	bool captureFrames = false;
	std::atomic_bool autoPilotEnabled;
	std::string play_mode = "single";
//	SimpleSerial *serialPort;
	boost::asio::io_service &io;
public:
    Robot(boost::asio::io_service &io, ICamera *pMainCamera, ICamera *pFrontCamera, ISerial*, bool master);
	bool Launch(const std::string &play_mode);
	~Robot();

	void SendFieldState();
	bool MessageReceived(const boost::array<char, BUF_SIZE>& buffer, size_t size);
	bool MessageReceived(const std::string & message);

};
