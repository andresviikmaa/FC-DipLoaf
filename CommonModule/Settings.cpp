#include "Settings.h"
#include <boost/program_options.hpp>
#include <iostream>

Settings::Settings():ConfigurableModule("Settings")
{
	ADD_STR_SETTING(name)
	ADD_STR_SETTING(ethernetIp)
	ADD_INT_SETTING(ethernetPort);
	ADD_STR_SETTING(mainCam);
	ADD_STR_SETTING(frontCam);
	ADD_BOOL_SETTING(master);
	ADD_STR_SETTING(boardcastSubnet)

	LoadSettings();
}

void Settings::LoadFromCommandLine(int argc, char* argv[]) {
	namespace po = boost::program_options;

	po::options_description desc("Allowed options");
	std::cout << desc << std::endl;


	po::variables_map config;

	po::store(po::parse_command_line(argc, argv, desc), config);
	po::notify(config);
	Settings settings;

	if (config.count("main-camera"))
		mainCam = config["main-camera"].as<std::string>();
	if (config.count("front-camera"))
		frontCam = config["front-camera"].as<std::string>();
	if (config.count("mainboard-ip"))
		ethernetIp = config["mainboard-ip"].as<std::string>();
	if (config.count("mainboard-port"))
		ethernetPort = config["mainboard-port"].as<int>();
	if (config.count("master"))
		master = config["master"].as<bool>();
	if (config.count("boardcast-subnet"))
		boardcastSubnet = config["boardcast-subnet"].as<std::string>();

	SaveSettings();
}

Settings::~Settings()
{
}
