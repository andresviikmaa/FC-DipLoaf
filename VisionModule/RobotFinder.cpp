#include "RobotFinder.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
RobotFinder::RobotFinder()
{
}


RobotFinder::~RobotFinder()
{
}



bool RobotFinder::Locate(cv::Mat &imgThresholded, cv::Mat &frameHSV, cv::Mat &frameBGR, std::vector<cv::Point2i> &objectCoords) {

	//try{
		cv::Point2d notValidPosition = cv::Point2d(-1.0, -1.0);
	
		int smallestArea = 300;
		int largestArea = 8000;
		cv::Point2d center(-1, -1);

		if (imgThresholded.rows == 0){
			std::cout << "Image thresholding has failed" << std::endl;
			return false;
		}

		cv::Mat dst(imgThresholded.rows, imgThresholded.cols, CV_8U, cv::Scalar::all(0));

		std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
		std::vector<cv::Vec4i> hierarchy;

		cv::Scalar blackColor(0, 0, 0);
		cv::Scalar whiteColor(255, 255, 255);
		cv::Scalar green(10, 255, 101);
		
		cv::findContours(imgThresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // Find the contours in the image

		if (contours.size() == 0){ //if no contours found
			return false;
		}
		cv::Point2d frameCenter = cv::Point2d(frameHSV.size()) / 2;
	
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			int blobArea = (int)(cv::contourArea(contours[i], false));
			if (blobArea >= smallestArea) {
				cv::Moments M = cv::moments(contours[i]);
				//cv::Rect bounding_rect = cv::boundingRect(contours[i]);
				//rectangle(frameBGR, bounding_rect.tl(), bounding_rect.br(), green, 1, 8, 0);
				if (M.m00 > 0.0001){
					objectCoords.push_back(cv::Point2d((M.m10 / M.m00), (M.m01 / M.m00)) - frameCenter);
				}
			}
		}
		return true;
	//}catch (cv::Exception ex){ return false; }
}