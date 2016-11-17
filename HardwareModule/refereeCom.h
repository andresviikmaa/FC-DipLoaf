#pragma once
#include "../CommonModule/Types.h"


class RefereeCom
{
public:
	RefereeCom(const std::string &name = "Referee");
	void giveCommand(GameMode command);
protected:
	void handleMessage(const std::string & message);
	virtual void sendAck(const std::string & message){};
	const char ALL_MARKER = 'X';

};
