#pragma once
#include "../CommonModule/Interfaces.h"
#include <functional>
#include <boost/thread/mutex.hpp>
#include <atomic>
#include "../CommonModule/ThreadedClass.h"
#include "opencv2/highgui.hpp"
#include <set>
/*
* No time to add QT support to OpenCV, have to make our own buttons (and dialogs)
* */

class Dialog : public IDisplay, public ThreadedClass {
public:
	Dialog(const std::string &m_Title, const cv::Size &windowSize = cv::Size(0, 0), const cv::Size &camSize = cv::Size(0, 0), int flags = CV_WINDOW_AUTOSIZE);
	int createButton(const std::string& bar_name, char shortcut, std::function<void()> const &);
	int Draw();
	void clearButtons();
	virtual void ShowImage(const cv::Mat &image, bool flip = true);
	virtual void ShowImage(const std::string &window, const cv::Mat &image, bool flip = true);
	void ClearDisplay();
	virtual void AddEventListener(IUIEventListener *pEventListener){
		auto tc = dynamic_cast<ThreadedClass*>(pEventListener);
		if (tc != NULL) {
			assert(!tc->name.empty());
			std::cout << "AddEventListener: " << tc->name << std::endl;
		}
		else{
			std::cout << "AddEventListener: " << "unknow class" << std::endl;
		}
		boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time
		m_EventListeners.push_back(pEventListener);
		std::cout << "count: " << m_EventListeners.size() << std::endl;
	};
	virtual void RemoveEventListener(IUIEventListener *pEventListener){
		auto tc = dynamic_cast<ThreadedClass*>(pEventListener);
		if (tc != NULL) {
			std::cout << "RemoveEventListener: " << tc->name << std::endl;
		}
		else{
			std::cout << "RemoveEventListener: " << "unknow class" << std::endl;
		}
		boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time
		for (auto it = m_EventListeners.begin(); it != m_EventListeners.end(); it++){
			if (*it == pEventListener){
				m_EventListeners.erase(it);
				break;
			}
		}
		std::cout << "count: " << m_EventListeners.size() << std::endl;
	};
	std::vector<IUIEventListener*> m_EventListeners;
	virtual void putText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color);
	virtual void putShadowedText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color);
	void Run();
	virtual ~Dialog();

protected:
	void mouseClicked(int event, int x, int y, int flag);
	std::atomic_int mouseX;
	std::atomic_int mouseY;

	cv::Mat display_empty;// (frameBGR.rows + 160, frameBGR.cols + 200, frameBGR.type(), cv::Scalar(0));
	cv::Mat display;// (frameBGR.rows + 160, frameBGR.cols + 200, frameBGR.type(), cv::Scalar(0));
//	cv::Mat cam1_roi, cam2_roi;// = display(cv::Rect(0, 0, frameBGR.cols, frameBGR.rows)); // region of interest
	//cv::Mat cam1_area, cam2_area;
	void KeyPressed(int key);
	cv::Size windowSize, camSize;
private:
    bool m_close = false;
    std::string m_title;
	std::vector<std::tuple<std::string, int, std::function<void()>>> m_buttons;
	std::map<std::string, std::tuple<cv::Point, std::string, double, cv::Scalar>> m_texts;
    int m_buttonHeight = 60; /* calculated automatically*/
	boost::mutex click_mutex;
	boost::mutex display_mutex;
	double fontScale; 
	std::string activeWindow = "main";
	std::set<std::string> windows;
};
