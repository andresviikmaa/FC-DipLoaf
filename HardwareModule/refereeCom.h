#pragma once
//#include "types.h"
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/ConfigurableModule.h"
#include "../CommonModule/Interfaces.h"
#include <queue>

class RefereeCom: public ConfigurableModule
{
public:
	RefereeCom(const std::string &name = "Referee");
	void giveCommand(GameMode command);

	virtual bool isTogglable() { return false; }
protected:
	void handleMessage(const std::string & message);
	virtual void sendAck(const std::string & message){};
	const char ALL_MARKER = 'X';
public:
	char FIELD_MARKER = 'A';
	char TEAM_MARKER = 'A';
	char ROBOT_MARKER = 'A';
private:
	void nextField();
	void nextTeam();
	void nextRobot();
};
