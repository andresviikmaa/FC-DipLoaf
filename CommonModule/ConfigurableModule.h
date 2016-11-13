#pragma once
#include "Interfaces.h"
class ConfigurableModule :
	public IConfigurableModule
{
public:
	ConfigurableModule(std::string sModuleName);
	virtual ~ConfigurableModule();
	 SettingsList &GetSettings() { return m_settings; };
	 std::string GetDebugInfo() { return std::string(""); }
private:
	SettingsList m_settings;
	std::string m_sModuleName;
protected:
	void AddSetting(const std::string& name, std::function<std::string()> const &, std::function<void(const std::string&)> const &);
	void LoadSettings();
	void SaveSettings();

};

#define ADD_BOOL_SETTING(setting) AddSetting(#setting, [this](){return this->setting ? "yes" : "no"; }, [this](const std::string& val){this->setting = val=="yes"; SaveSettings();});
#define ADD_STR_SETTING(setting) AddSetting(#setting, [this](){return this->setting;}, [this](const std::string& val){this->setting = val;});
#define ADD_INT_SETTING(setting) AddSetting(#setting, [this](){return std::to_string(this->setting);}, [this](const std::string& val){this->setting = atoi(val.c_str());});


