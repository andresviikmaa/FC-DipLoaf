#include "CoilBoard.h"
#include <chrono>
#include "../CommonModule/Types.h"

#define TRIBBLER_QUEUE_SIZE 30
#define TRIBBLER_STATE_THRESHOLD 16

void CoilBoard::SetBallInTribbler(bool inTribbler)
{
	if (!ballInTribbler && inTribbler)
		ballCatchTime = boost::posix_time::microsec_clock::local_time();
	if (ballInTribbler && !inTribbler)
		ballLostTime = boost::posix_time::microsec_clock::local_time();
	ballInTribbler = inTribbler;

}

long CoilBoard::BallInTribblerTime(){
	if (ballInTribbler) return (boost::posix_time::microsec_clock::local_time() - ballCatchTime).total_milliseconds();
	else return 0L;
}
long CoilBoard::BallNotInTribblerTime() {
	if (!ballInTribbler) return (boost::posix_time::microsec_clock::local_time() - ballLostTime).total_milliseconds();
	else return 0L;
}


bool CoilBoard::KickAllowed(int force){
	std::cout << "kickforce " << force << std::endl;
	boost::posix_time::ptime time2 = boost::posix_time::microsec_clock::local_time();
	//std::cout << (afterKickTime - time2).total_milliseconds() << std::endl;
	if (!kickAllowed) {
		std::cout << "coil not ready, not kicking" << std::endl;
		return false;
	}
	if ((time2 - afterKickTime).total_milliseconds() < 1500) return false;
	//WriteString("k800\n");
	kickForce = force; // set flag, so that we do not corrupt writing in Run method
	//forcedNotInTribbler = true;

	afterKickTime = time2; //reset timer
	return true;
}

