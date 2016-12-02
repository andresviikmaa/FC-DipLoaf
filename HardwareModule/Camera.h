#pragma  once
#include "../CommonModule/Types.h"
#include "../CommonModule/ThreadedClass.h"
#include "../CommonModule/Interfaces.h"
#include <opencv2/videoio.hpp>

class Camera: public ICamera, public ThreadedClass
{
	enum {
		CAPTURE_FRAME1 = 0,
		RETURN_FRAME1,
		CAPTURE_FRAME2,
		RETURN_FRAME2
	};
private:
    cv::Mat frame, frame1, frame2, lastframe, buffer, frame_roi, frame1_roi, frame2_roi, maskedImage, mask;
	std::atomic_bool bCaptureNextFrame;
	std::atomic_bool bCaptureFrame1;
	cv::Mat* m_pFrame = &frame1;
	cv::VideoCapture *cap;
	cv::Size frameSize;
	cv::Point2d cameraOrgin;
	cv::Rect roi = cv::Rect(175, 60, 960, 960);
	double fps;
	int frames = 0;
	double time = 0;
	int frameCount;
	std::string name;
protected:
	void Run();
	void Start() {
		bCaptureFrame1 = true;
		bCaptureNextFrame = true;
		ThreadedClass::Start();
	}
	std::atomic_bool paused;
public:
    Camera(const std::string &name, const std::string &device);
	//Camera(const std::string &name, int device);
	cv::Mat & Capture(bool bFullFrame = false);
	cv::Mat & GetLastFrame(bool bFullFrame = false);
	const cv::Mat & CaptureHSV();
	virtual void TogglePlay(){
		paused = !paused;
	};
    virtual ~Camera(){ 
		WaitForStop();
		cap->release();
		delete cap;
	}
	const std::string & getName() {
		return name;
	};
	virtual cv::Size GetFrameSize(bool bFullFrame = false){
		return !bFullFrame ? cv::Size(roi.width, roi.height) : frameSize;
	};
	virtual double GetFPS() {
		return fps;
	}

	virtual HSVColorRange GetObjectThresholds(int index, const std::string &name);
};
