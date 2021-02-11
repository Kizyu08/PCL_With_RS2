#pragma once

#include "cv_dnn.hpp"
#include "libstest.hpp"
#include "points_position_detector.h"
#include <atomic>
#include <thread>

#define UNITYCALLCONV __stdcall
#define UNITYEXPORT __declspec(dllexport)

using namespace libstest;

extern "C"
{
	points_position_detector ppd;
	std::vector<myBox> boxes;
	cv::Mat image;
	float* boxData;

	//detector
	UNITYEXPORT void UNITYCALLCONV startDetector(const char*);
	UNITYEXPORT void UNITYCALLCONV stopDetector();
	
	//image
	UNITYEXPORT void UNITYCALLCONV getImage();
	UNITYEXPORT int UNITYCALLCONV getWidth();
	UNITYEXPORT int UNITYCALLCONV getHeight();
	UNITYEXPORT int UNITYCALLCONV getDataSize();
	UNITYEXPORT void UNITYCALLCONV getImageData(unsigned char* data);
	
	//boxes
	UNITYEXPORT void UNITYCALLCONV getBoxes();
	UNITYEXPORT int UNITYCALLCONV getBoxesCount();
	UNITYEXPORT float* UNITYCALLCONV getBoxesData();

	//dll
	//UNITYEXPORT void UNITYCALLCONV initThis();
}

std::string GetIniPath();
