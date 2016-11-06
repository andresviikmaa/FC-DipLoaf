#include "Settings.h"



Settings::Settings():ConfigurableModule("Settings")
{
	ADD_STR_SETTING(name)
	ADD_STR_SETTING(ethernetIp)
	ADD_INT_SETTING(ethernetPort);
}


Settings::~Settings()
{
}
