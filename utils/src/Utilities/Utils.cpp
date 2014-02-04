/*
 * Utils.cpp
 *
 *  Created on: 21 Nov 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#include "Utilities/Utils.h"

#include "opencv2/highgui/highgui.hpp"

#include <stdexcept>

namespace Tagger3D {

Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}


int Utils::visualizeKeypoints(const cv::Mat &img, std::vector<cv::KeyPoint> &keys){

    cv::Mat imgWithKeys;    // save image with visualised keypoints
    try {
		cv::drawKeypoints(img, keys, imgWithKeys);
		cv::imshow("KeyPoints Visualisation", imgWithKeys);
    } catch(std::runtime_error &e) {
        return 0;
    }
    cv::waitKey(0);
    return 1;
}

std::string Utils::cvType2Str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

} /* namespace Tagger3D */
