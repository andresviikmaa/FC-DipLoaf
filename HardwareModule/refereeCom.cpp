#include "refereeCom.h"
#include "../CommonModule/GameState.h"
#include "../CommonModule/RobotState.h"
#include <iostream>
extern RobotState gRobotState;

RefereeCom::RefereeCom(const std::string &name)
{

}


void RefereeCom::handleMessage(const std::string & message){
	//TODO: update m_pFieldState->gameMode from here directly, add missing start commands there
	std::cout << "referee command: " << message << std::endl;
	std::string command = message.substr(0, 12);
	if (command.length() == 12 && command.at(0) == 'a' && command.at(1) == gRobotState.FIELD_MARKER && (command.at(2) == ALL_MARKER || command.at(2) == gRobotState.ROBOT_MARKER)) {
		if (command.at(2) == gRobotState.ROBOT_MARKER) sendAck("a" + std::string(1, gRobotState.FIELD_MARKER) + std::string(1, gRobotState.ROBOT_MARKER) + "ACK------");
		command = command.substr(3);
		if (command == "START----") gRobotState.gameMode = GAME_MODE_START_PLAY;
		else if (command == "STOP-----") {
			gRobotState.gameMode = GAME_MODE_END_HALF;
		}
		else if (command == "PLACEDBAL") gRobotState.gameMode = GAME_MODE_PLACED_BALL;
		else if (command == "ENDHALF--") gRobotState.gameMode = GAME_MODE_END_HALF;
		else if (command.at(0) == gRobotState.TEAM_MARKER) {
			command = command.substr(1);
			if (command == "KICKOFF-") gRobotState.gameMode = GAME_MODE_START_OUR_KICK_OFF;
			else if (command == "IFREEK--") gRobotState.gameMode = GAME_MODE_START_OUR_INDIRECT_FREE_KICK;
			else if (command == "DFREEK--") gRobotState.gameMode = GAME_MODE_START_OUR_FREE_KICK;
			else if (command == "GOALK---") gRobotState.gameMode = GAME_MODE_START_OUR_GOAL_KICK;
			else if (command == "THROWIN-") gRobotState.gameMode = GAME_MODE_START_OUR_THROWIN;
			else if (command == "CORNERK-") gRobotState.gameMode = GAME_MODE_START_OUR_CORNER_KICK;
			else if (command == "PENALTY-") gRobotState.gameMode = GAME_MODE_START_OUR_PENALTY;
			else if (command == "GOAL----") gRobotState.gameMode = GAME_MODE_START_OUR_GOAL;
			else if (command == "CARDY---") gRobotState.gameMode = GAME_MODE_START_OUR_YELLOW_CARD;
		}
		else {
			command = command.substr(1);
			if (command == "KICKOFF-") gRobotState.gameMode = GAME_MODE_START_OPPONENT_KICK_OFF;
			else if (command == "IFREEK--") gRobotState.gameMode = GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK;
			else if (command == "DFREEK--") gRobotState.gameMode = GAME_MODE_START_OPPONENT_FREE_KICK;
			else if (command == "GOALK---") gRobotState.gameMode = GAME_MODE_START_OPPONENT_GOAL_KICK;
			else if (command == "THROWIN-") gRobotState.gameMode = GAME_MODE_START_OPPONENT_THROWIN;
			else if (command == "CORNERK-") gRobotState.gameMode = GAME_MODE_START_OPPONENT_CORNER_KICK;
			else if (command == "PENALTY-") gRobotState.gameMode = GAME_MODE_START_OPPONENT_PENALTY;
			else if (command == "GOAL----") gRobotState.gameMode = GAME_MODE_START_OPPONENT_GOAL;
			else if (command == "CARDY---") gRobotState.gameMode = GAME_MODE_START_OPPONENT_YELLOW_CARD;
		}
	}
}
