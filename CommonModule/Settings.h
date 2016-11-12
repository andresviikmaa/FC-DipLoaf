#pragma once
#include "ConfigurableModule.h"
class Settings :
	public ConfigurableModule
{
public:
	std::string name;
	std::string ethernetIp;
	int ethernetPort;
	std::string frontCam;
	std::string mainCam;
	bool master;
	Settings();
	~Settings();
	void LoadFromCommandLine(int argc, char* argv[]);
};

