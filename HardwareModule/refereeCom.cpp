#include "refereeCom.h"
#include "../CommonModule/GameState.h"
#include "../CommonModule/RobotState.h"
#include <iostream>
extern RobotState gRobotState;

RefereeCom::RefereeCom(const std::string &name)
{

}


void RefereeCom::handleMessage(const std::string & message){
	std::cout << "referee command: " << message << std::endl;
	std::string command = message.substr(0, 12);
	if (command.length() == 12 && command.at(0) == 'a' && command.at(1) == gRobotState.FIELD_MARKER && (command.at(2) == ALL_MARKER || command.at(2) == gRobotState.ROBOT_MARKER)) {
//		if (command.at(2) == gRobotState.ROBOT_MARKER) sendAck("a" + std::string(1, gRobotState.FIELD_MARKER) + std::string(1, gRobotState.ROBOT_MARKER) + "ACK------");
		bool sendAck = command.at(2) == gRobotState.ROBOT_MARKER;
		command = command.substr(3);


		if (gRobotState.runMode = ROBOT_MODE_1VS1) return handleMessage1vs1(command, sendAck);
		else if (gRobotState.runMode = ROBOT_MODE_2VS2) return handleMessage2vs2(command, sendAck);
		else std::cout << "Ignoring referee message in idle mode" << std::endl;
	}
}

void RefereeCom::handleMessage1vs1(const std::string & command, bool sendAck){
	if (command == "START----") {
		gRobotState.gameMode = GAME_MODE_START_PLAY;
	}
	else if (command == "STOP-----") {
		gRobotState.gameMode = GAME_MODE_END_HALF;
	}
	else if (command == "PING-----") {
		; // just send ack
	}
	if (sendAck){
		this->sendAck("a" + std::string(1, gRobotState.FIELD_MARKER) + std::string(1, gRobotState.ROBOT_MARKER) + "ACK------");
	}

}

void RefereeCom::handleMessage2vs2(const std::string & command, bool sendAck){
	if (command == "S") gRobotState.gameMode = gRobotState.pendingGameMode;
	else if (command == "H") gRobotState.gameMode = GAME_MODE_END_HALF;
	else if (command == "B") gRobotState.pendingGameMode = GAME_MODE_PLACED_BALL;
	else if (command == "E") gRobotState.pendingGameMode = GAME_MODE_END_HALF;
	else if (command == "A") assert(false); //TODO: send ping back
	else {
		bool ourTeamA = gRobotState.TEAM_MARKER == 'A';
		char c = command[1];
		if (!ourTeamA) { 
			c += 32; // convert command to upper case
			ourTeamA = !ourTeamA; // and pretend that we are team A
		}
		if (c == 'K') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_KICK_OFF : GAME_MODE_START_OPPONENT_KICK_OFF;
		else if (c == 'I') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_INDIRECT_FREE_KICK : GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK;
		else if (c == 'D') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_FREE_KICK : GAME_MODE_START_OPPONENT_FREE_KICK;
		else if (c == 'P') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_PENALTY : GAME_MODE_START_OPPONENT_PENALTY;
		else if (c == 'G') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_GOAL : GAME_MODE_START_OPPONENT_GOAL;
		else if (c == 'Y') gRobotState.pendingGameMode = ourTeamA ? GAME_MODE_START_OUR_YELLOW_CARD : GAME_MODE_START_OPPONENT_YELLOW_CARD;
	}
}
