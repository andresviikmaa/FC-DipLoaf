#include "../CommonModule/Types.h"
#include "../CommonModule/Interfaces.h"
#include <atomic>
#include <boost/thread/mutex.hpp>
#include "../CommonModule/UdpServer.h"
#include <boost/asio.hpp>

enum STATE
{
	STATE_NONE = 0,
	STATE_AUTOCALIBRATE,
	STATE_CALIBRATE,
	STATE_LAUNCH,
	STATE_SELECT_GATE,
	STATE_RUN,
	STATE_SETTINGS,
	STATE_REMOTE_CONTROL,
	STATE_MANUAL_CONTROL,
	STATE_DANCE,
	STATE_TEST,
	STATE_MOUSE_VISION,
	STATE_DISTANCE_CALIBRATE,
	STATE_GIVE_COMMAND,
	STATE_END_OF_GAME /* leave this last*/
};

class Robot: public UdpServer {
private:
	IVisionModule *m_pMainVision = NULL;
	IVisionModule *m_pFrontVision = NULL;
	ICommunicationModule *m_pComModule = NULL;
	IStateMachine *m_pAutoPilot = NULL;


    //STATE state = STATE_NONE;
    std::atomic<STATE> state;
	std::atomic<STATE> last_state;
	//void InitHardware();
	//void InitSimulator(bool master, const std::string game_mode);
//	void initWheels();
//	void initCoilboard();
	//void initRefCom();

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

    int GetState() {
        return state;
    }
	int GetLastState() {
		return last_state;
	}
    void SetState(STATE new_state) {
		last_state = (STATE)state;
        state = new_state;
    }
	void SendFieldState();
};
