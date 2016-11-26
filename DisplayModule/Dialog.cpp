#include "Dialog.h"
#include "opencv2/imgproc.hpp"
/*
#define WINDOW_WIDTH 976
#define WINDOW_HEIGHT 840

#define CAM_WIDTH 720
#define CAM_HEIGHT 720
*/

Dialog::Dialog(const std::string &title, const cv::Size &ptWindowSize, const cv::Size &ptCamSize, int flags/* = CV_WINDOW_AUTOSIZE*/)
	: windowSize(ptWindowSize), camSize(ptCamSize), ThreadedClass("Dialog")
{
	cv::Size windowSizeDefault = cv::Size(1024, 768);

	if (windowSize != cv::Size(0, 0)) {
		double scale = (double)windowSize.width / (double)windowSizeDefault.width;
		camSize = cv::Size((int)((double)camSize.width * scale), (int)((double)camSize.height * scale));
	} 
	else {
		windowSize = cv::Size((int)((double)camSize.width / 0.7), (int)((double)camSize.width / 0.7));
	}
	fontScale = (double)windowSize.height / 1024;
    m_title = title;
    int baseLine;

	m_buttonHeight = cv::getTextSize("Ajig6", cv::FONT_HERSHEY_DUPLEX, fontScale, 1, &baseLine).height * 2;

	/*
    m_buttonHeight = cv::getTextSize("Ajig6", cv::FONT_HERSHEY_DUPLEX, fontScale, 1, &baseLine).height * 2;
	
	cv::namedWindow(m_title, CV_WINDOW_AUTOSIZE);
	//cvSetWindowProperty(m_title.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	cv::moveWindow(m_title, 0, 0);
	cv::setMouseCallback(m_title, [](int event, int x, int y, int flags, void* self) {
		for (auto pListener : ((Dialog*)self)->m_EventListeners){
			if (pListener->OnMouseEvent(event, (float)x / ((Dialog*)self)->camSize.x, (float)y / ((Dialog*)self)->camSize.y, flags)) {
				return; // event was handled
			}
		}

		((Dialog*)self)->mouseX = x;
		((Dialog*)self)->mouseY = y;
		if (event == cv::EVENT_LBUTTONUP){
			((Dialog*)self)->mouseClicked(x, y);
		}
	}, this);
*/

	display_empty = cv::Mat(windowSize, CV_8UC3, cv::Scalar(0));
	display = cv::Mat(windowSize, CV_8UC3, cv::Scalar(0));
	Start();

};
Dialog::~Dialog(){
	stop_thread = true;
	WaitForStop();
}
void Dialog::ShowImage(const std::string &window, const cv::Mat &image, bool flip){
	boost::mutex::scoped_lock lock(display_mutex); //allow one command at a time
	if (windows.size() == 0){
		activeWindow = window;
	}
	if (windows.find(window) == windows.end()) {
		std::string name(window);
		createButton(name, '-', [&, name]{
			activeWindow = name;
		});
		windows.insert(name);
	}
	if (window == activeWindow){
		image.copyTo(display);
		camSize = image.size();
		//resize(image, display, display.size());
	}

}

void Dialog::ShowImage(const cv::Mat &image, bool flipX) {
	boost::mutex::scoped_lock lock(display_mutex); //allow one command at a time

#ifdef VIRTUAL_FLIP
		if(flipX)
			cv::flip(image, cam1_area, 1);
		else
			image.copyTo(cam1_area);
#else
	camSize = image.size();
	//image.copyTo(display);
	resize(image, display, display.size());


#endif

	//	resize(image, cam_area, cv::Size(CAM_WIDTH, CAM_HEIGHT));//resize image
	//resize(image, display, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));//resize image
}

int Dialog::createButton(const std::string& bar_name, char shortcut, std::function<void()> const & on_change){
	boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time
	m_buttons.push_back(std::make_tuple(bar_name, shortcut, on_change));
	return 0;
};

void Dialog::clearButtons() {
	boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time
	m_buttons.clear();
}

void Dialog::ClearDisplay() {
//	boost::mutex::scoped_lock lock(mutex); //allow one command at a time
//	display_empty.copyTo(display);
}
void Dialog::putText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) {
	boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time

	if (pos.x < 0) pos.x = display.size().width + pos.x;
	if (pos.y < 0) pos.y = display.size().height + pos.y;
	std::string key = std::to_string(pos.x) + "_" + std::to_string(pos.y);
	m_texts[key] = std::make_tuple(pos, text, fontScale, color);
	//cv::putText(display, text, pos, cv::FONT_HERSHEY_DUPLEX, fontScale, color);
}

void Dialog::putShadowedText(const std::string &text, cv::Point pos, double fontScale, cv::Scalar color) {
	putText(text, cv::Point(pos.x + 0.2, pos.y + 0.2), fontScale, color);
	putText(text, pos, fontScale, cv::Scalar(0, 0,0));
}


int Dialog::Draw() {
	{
		boost::mutex::scoped_lock lock(display_mutex); //allow one command at a time
		display.copyTo(display_empty);
	}

	{
		boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time

		int i = 0;
		for (const auto& button : m_buttons) {
			++i;
			cv::putText(display_empty, std::get<0>(button), cv::Point(31, (i)*m_buttonHeight), cv::FONT_HERSHEY_DUPLEX, fontScale, cv::Scalar(0, 0, 0));
			cv::putText(display_empty, std::get<0>(button), cv::Point(30, (i)*m_buttonHeight), cv::FONT_HERSHEY_DUPLEX, fontScale, cv::Scalar(255, 255, 255));
		}
		for (const auto& text : m_texts) {
			cv::putText(display_empty, std::get<1>(text.second), std::get<0>(text.second), cv::FONT_HERSHEY_DUPLEX, std::get<2>(text.second), std::get<3>(text.second));

		}
		cv::imshow(m_title, display_empty);
	}
	//display_empty.copyTo(display);

	return 0;
};
void Dialog::KeyPressed(int key){

	if (key == '-') return;
	boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time
	for (auto btn : m_buttons){
		if(std::get<1>(btn) == key){
			std::get<2>(btn)();
			return;
		}
	}
}

void Dialog::mouseClicked(int event, int x, int y, int flag) { 
	boost::mutex::scoped_lock lock(click_mutex); //allow one command at a time

	mouseX = x;
	mouseY = y;
	cv::Point2d scaled;
	cv::Size size;
	cv::Size target;


	size = display.size();
	target = camSize;

	scaled.x = (float)x / size.width * target.width;
	scaled.y = (float)y / size.height * target.height;

	for (auto pListener : m_EventListeners){

		if (pListener->OnMouseEvent(event, scaled.x, scaled.y, flag)) {
			return; // event was handled
		}
	}
	

	if (event == cv::EVENT_LBUTTONUP){
		unsigned int index = (int)(round((float)y / m_buttonHeight) - 1);
		if (index < m_buttons.size()){
			auto button = m_buttons[index];
			std::get<2>(button)();
			m_close = true;
		}
	}

}
void Dialog::Run(){
	cv::namedWindow(m_title, CV_WINDOW_AUTOSIZE);
	//cvSetWindowProperty(m_title.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	cv::moveWindow(m_title, 0, 0);
	cv::setMouseCallback(m_title, [](int event, int x, int y, int flags, void* self) {
		((Dialog*)self)->mouseClicked(event, x, y, flags);
	}, this);

	while (!stop_thread) {
		try {
			Draw();
			int key = cv::waitKey(10);
			if (key == 27) stop_thread = true;
			KeyPressed(key);
		}
		catch (std::exception &e) {
			std::cout << "Dialog::Run, error: " << e.what() << std::endl;
		}
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	cv::waitKey(10);
}
