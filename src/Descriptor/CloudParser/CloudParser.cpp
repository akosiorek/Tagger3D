/*
 * Tagger3D : CloudParser.cpp
 *
 *  Created on: 	31 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "CloudParser.h"

namespace Tagger3D {

CloudParser::CloudParser() {

	DEBUG(logger, "Creating CloudParser")
}

CloudParser::~CloudParser() {

	DEBUG(logger, "Deleting CloudParser");
}

cv::Mat CloudParser::parsePFH(const PfhCloud::Ptr &cloud) {

	TRACE(logger, "parsePFH: parsing");
	// TODO size does not equal the number of rows
	int size = cloud->size();
	cv::Mat mat( size, PFH125, CV_32FC1 );
	for(int i = 0; i < size; i++) {
		float* dPtr = mat.ptr<float>(i);
		float* hPtr = cloud->points[i].histogram;
		for(int j = 0; j < PFH125; j++)
			dPtr[j] = hPtr[j];
	}
	TRACE(logger, "parse PFH: done parsing");
	return mat;
}

std::vector<cv::Mat> CloudParser::parsePFH(const PfhVec &clouds) {

	std::vector<cv::Mat> vec;
	for(const auto &cloud : clouds) {

		vec.push_back( parsePFH( cloud ) );
	}
	return vec;
}

cv::Mat CloudParser::parseFPFH(const FpfhCloud::Ptr &cloud) {

	TRACE(logger, "parseFPFH: parsing");
	// TODO size does not equal the number of rows
	int size = cloud->size();
	cv::Mat mat( size, FPFH33, CV_32FC1 );
	for(int i = 0; i < size; i++) {
		float* dPtr = mat.ptr<float>(i);
		float* hPtr = cloud->points[i].histogram;
		for(int j = 0; j < FPFH33; j++)
			dPtr[j] = hPtr[j];
	}
	TRACE(logger, "parse FPFH: done parsing; mat size =  " << mat.size());
	return mat;
}

std::vector<cv::Mat> CloudParser::parseFPFH(const FpfhVec &clouds) {

	TRACE(logger, "parseFPFH: Starting batch processing");
	std::vector<cv::Mat> vec;
	for(const auto &cloud : clouds) {

		vec.push_back( parseFPFH( cloud ) );
	}
	TRACE(logger, "parse FPFH: Finished batch processing; vec size = " << vec.size());
	return vec;
}

} /* namespace Tagger3D */
