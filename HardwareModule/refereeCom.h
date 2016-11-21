#pragma once
#include "../CommonModule/Types.h"


class RefereeCom
{
public:
	RefereeCom(const std::string &name = "Referee");
protected:
	void handleMessage(const std::string & message);
	void handleMessage1vs1(const std::string & message, bool sendAck = false);
	void handleMessage2vs2(const std::string & message, bool sendAck = false);
	virtual void sendAck(const std::string & message){};
	const char ALL_MARKER = 'X';
	GameMode2vs2 pendingGameMode = GAME_MODE_END_HALF;
};
