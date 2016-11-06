#pragma once
#include "ConfigurableModule.h"
class Settings :
	public ConfigurableModule
{
public:
	std::string name;
	std::string ethernetIp;
	int ethernetPort;
	Settings();
	~Settings();
};

