#pragma once
#include "Types.h"
#include <string>
#include <map>
#include <functional>
#include <opencv2/core.hpp>

//maybe use std::vector instead
typedef std::map<std::string, std::tuple<std::function<std::string()>, std::function<void()>>> SettingsList;

class ICamera
{
public:
	virtual cv::Mat & Capture(bool bFullFrame = false) = 0;
	virtual cv::Size GetFrameSize(bool bFullFrame = false) = 0;
	virtual double GetFPS() = 0;
	virtual cv::Mat & GetLastFrame(bool bFullFrame = false) = 0;
	virtual void TogglePlay() = 0;
};

class IConfigurableModule {
public:
	virtual SettingsList &GetSettings() = 0;
	virtual std::string GetDebugInfo() = 0;
};

class IUIEventListener {
public:
	// xy coordinates are from 0...1.0...
	virtual bool OnMouseEvent(int event, float x, float y, int flags, bool bMainArea) { return false; };
	virtual void OnKeyPress(char key) {};
};


class IDisplay {
public:
	virtual int createButton(const std::string& bar_name, char shortcut, std::function<void()> const &) = 0;
	virtual int Draw() = 0;
	virtual void clearButtons() = 0;
	virtual void ShowImage(const cv::Mat &image, bool main = true, bool flip = true) = 0;
	virtual void AddEventListener(IUIEventListener *pEventListener) = 0;
	virtual void RemoveEventListener(IUIEventListener *pEventListener) = 0;
	virtual void putText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) = 0;
	virtual void putShadowedText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) = 0;
	virtual void SwapDisplays() = 0;
	virtual void ToggleDisplay() = 0;
	virtual ~IDisplay() {};
};



class IVisionModule {
public:
	virtual const cv::Mat & GetFrame() = 0;
	virtual void ProcessFrame(double dt) = 0;
};

class IStateMachine {
public:
	virtual void Step(double dt) = 0;
};
