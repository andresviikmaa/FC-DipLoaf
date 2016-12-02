#include "refereeCom.h"
#include "../CommonModule/GameState.h"
#include "../CommonModule/RobotState.h"
#include <iostream>
extern RobotState gRobotState;
const size_t MIN_COMMAND_LENGTH = 5;

#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */

/*
* The width of the CRC calculation and result.
* Modify the typedef for a 16 or 32-bit CRC standard.
*/

#define WIDTH  (8 * sizeof(uchar))
#define TOPBIT (1 << (WIDTH - 1))

uchar
crcSlow(uchar message[], int nBytes)
{
	uchar  remainder = 0;


	/*
	* Perform modulo-2 division, a byte at a time.
	*/
	for (int byte = 0; byte < nBytes; ++byte)
	{
		/*
		* Bring the next byte into the remainder.
		*/
		remainder ^= (message[byte] << (WIDTH - 8));

		/*
		* Perform modulo-2 division, a bit at a time.
		*/
		for (uchar bit = 8; bit > 0; --bit)
		{
			/*
			* Try to divide the current data bit.
			*/
			if (remainder & TOPBIT)
			{
				remainder = (remainder << 1) ^ POLYNOMIAL;
			}
			else
			{
				remainder = (remainder << 1);
			}
		}
	}

	/*
	* The final remainder is the CRC result.
	*/
	return (remainder);

}   /* crcSlow() */

RefereeCom::RefereeCom(const std::string &name)
{

}


void RefereeCom::handleMessage(const std::string & message){

	if (gRobotState.runMode == ROBOT_MODE_1VS1) {
		std::cout << "referee command: " << message << std::endl;
		std::string command = message.substr(0, 12);
		if (command.length() == 12 && command.at(0) == 'a' && command.at(1) == gRobotState.FIELD_MARKER && (command.at(2) == ALL_MARKER || command.at(2) == gRobotState.ROBOT_MARKER)) {
			//		if (command.at(2) == gRobotState.ROBOT_MARKER) sendAck("a" + std::string(1, gRobotState.FIELD_MARKER) + std::string(1, gRobotState.ROBOT_MARKER) + "ACK------");
			bool sendAck = command.at(2) == gRobotState.ROBOT_MARKER;
			command = command.substr(3);


			return handleMessage1vs1(command, sendAck);
		}
		else {
			std::cout << "Unable to parse referee message " << message << std::endl;
		}
	}
	else if (gRobotState.runMode == ROBOT_MODE_2VS2) return handleMessage2vs2(message, false);
	else std::cout << "Ignoring referee message in idle mode" << std::endl;
}

void RefereeCom::handleMessage1vs1(const std::string & command, bool sendAck){
	std::cout << "handleMessage1vs1 " << command << std::endl;
	if (command == "START----") {
		gRobotState.gameMode = GAME_MODE_START_PLAY;
	}
	else if (command == "STOP-----") {
		gRobotState.gameMode = GAME_MODE_END_HALF;
	}
	else if (command == "PING-----") {
		; // just send ack
	}
	else if (command == "ACK------") {
		return; // ignore ack
	}
	if (sendAck){
		this->sendAck("a" + std::string(1, gRobotState.FIELD_MARKER) + std::string(1, gRobotState.ROBOT_MARKER) + "ACK------");
	}

}

void RefereeCom::handleMessage2vs2(const std::string & message, bool sendAck){
	for (const auto& e : message)
		command_buffer.push_back(e);

	char b;
	char field;
	char robot;
	char command = '-';
	uchar crcR;
	uchar crcC;
	while (command_buffer.size() >= MIN_COMMAND_LENGTH){ // min for 2vs2 commands
		b = command_buffer[0];
		if (b != 'a'){
			command_buffer.pop_front();
			continue; // skip until start bit
		}
		field = command_buffer[1];
		robot = command_buffer[2];
		command = command_buffer[3];
		crcR = command_buffer[4];
		crcC = crcSlow(&command_buffer[0], 4);
		//if (crcR != crcC){
		//	std::cout << "invalid crc recieved, skiping command" << std::endl;
		//}
		command_buffer.pop_front();
		command_buffer.pop_front();
		command_buffer.pop_front();
		command_buffer.pop_front();
		command_buffer.pop_front();
		if (crcR != crcC){
			continue;
		}

		if (field != gRobotState.FIELD_MARKER) continue;
		handleCommand2vs2(robot, command);

	}
}
void RefereeCom::handleCommand2vs2(char robot, char command){
	std::cout << "handleCommand2vs2: " << robot << command << std::endl;
	if (command == 'S') gRobotState.gameMode = gRobotState.pendingGameMode;
	else if (command == 'H') gRobotState.gameMode = GAME_MODE_END_HALF;
	else if (command == 'B') gRobotState.pendingGameMode = GAME_MODE_PLACED_BALL;
	else if (command == 'E') gRobotState.pendingGameMode = GAME_MODE_END_HALF;
	else if (command == 'A') assert(false); //TODO: send ping back
	else {
		bool ourTeamA = gRobotState.TEAM_MARKER == 'A';
		char c = command;
		/*
		if (!ourTeamA) { 
			c += 32; // convert command to upper case
			ourTeamA = !ourTeamA; // and pretend that we are team A
		}
		*/
		if (c == 'K') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_KICK_OFF : GAME_MODE_START_OPPONENT_KICK_OFF;
		else if (c == 'I') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_INDIRECT_FREE_KICK : GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK;
		else if (c == 'D') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_FREE_KICK : GAME_MODE_START_OPPONENT_FREE_KICK;
		else if (c == 'P') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_PENALTY : GAME_MODE_START_OPPONENT_PENALTY;
		else if (c == 'G') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_GOAL : GAME_MODE_START_OPPONENT_GOAL;
		else if (c == 'Y') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_YELLOW_CARD : GAME_MODE_START_OPPONENT_YELLOW_CARD;

		else if (c == 'k') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_KICK_OFF : GAME_MODE_START_OPPONENT_KICK_OFF;
		else if (c == 'i') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_INDIRECT_FREE_KICK : GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK;
		else if (c == 'd') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_FREE_KICK : GAME_MODE_START_OPPONENT_FREE_KICK;
		else if (c == 'p') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_PENALTY : GAME_MODE_START_OPPONENT_PENALTY;
		else if (c == 'g') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_GOAL : GAME_MODE_START_OPPONENT_GOAL;
		else if (c == 'y') gRobotState.pendingGameMode = !ourTeamA ? GAME_MODE_START_OUR_YELLOW_CARD : GAME_MODE_START_OPPONENT_YELLOW_CARD;
	}
	if (robot == gRobotState.ROBOT_MARKER) {
	
		uchar ack[5] = { 'a', gRobotState.FIELD_MARKER, gRobotState.ROBOT_MARKER, 'A', 0 };
		ack[4] = crcSlow(ack, 4);
		this->sendAck(std::string(reinterpret_cast<char*>(ack), 5));
	}
}
