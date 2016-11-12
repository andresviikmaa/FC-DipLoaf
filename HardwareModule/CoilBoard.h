#pragma  once
#include <boost/timer/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
class CoilBoard
{
private:
	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime waitTime = time;
	boost::posix_time::time_duration waitDuration;
	boost::posix_time::ptime afterKickTime = time;
	//boost::posix_time::time_duration afterKickDuration;
	bool kickAllowed;
	int kickForce ;
	int ballInTribblerCount;
	//bool forcedNotInTribbler = false;
	int lastTribblerSpeed=0;
public:
	CoilBoard(){
		kickForce = 0;
		ballInTribbler = false;
		ballInTribblerCount = 0;
		kickAllowed = true;
		ballCatchTime = boost::posix_time::microsec_clock::local_time();
		ballLostTime = boost::posix_time::microsec_clock::local_time();

	};
	bool KickAllowed(int force);
	bool BallInTribbler(bool wait=false) { 
		if(wait) {
			if (ballInTribbler) { return BallInTribblerTime() > 300; }
			else return BallNotInTribblerTime() < 300;
			
		}else {
			return ballInTribbler; 
		}
	}
	long BallInTribblerTime(); 
	long BallNotInTribblerTime();

	void SetBallInTribbler(bool inTribbler);

protected:
	bool ballInTribbler;
	boost::posix_time::ptime ballCatchTime;
	boost::posix_time::ptime ballLostTime;
	std::string last_message;


};
