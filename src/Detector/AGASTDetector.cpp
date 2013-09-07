/*
 * AGASTDetector.cpp
 *
 *   Created on: 28 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "AGASTDetector.h"

#include <pcl/search/kdtree.h>
#include <assert.h>

namespace Tagger3D {

AGASTDetector::AGASTDetector(const std::map<std::string, std::string> &configMap) :
	Detector(configMap),
	width(getParam<int>(widthKey)),
	height(getParam<int>(heightKey)),
	threshold(getParam<float>(thresholdKey)),
	bmax(getParam<float>(bmaxKey))
	{}

AGASTDetector::~AGASTDetector() {}

bool AGASTDetector::createAGASTDetector() {

	TRACE(logger, "createAGASTDetector: Starting");
	decltype(agastDetector) detector( new pcl::keypoints::agast::OastDetector9_16<pcl::PointXYZRGB, pcl::PointWithScale>() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );


	AGASTDetector = std::move( detector );

	TRACE(logger, "createAGASTDetector: Finished");
	return true;
}

ScaleCloud::Ptr AGASTDetector::detect(const ColorCloud::Ptr &cloud) {

	TRACE(logger, "detect: Starting");
	ScaleCloud::Ptr keyPoints(new ScaleCloud() );
	AGASTDetector->setSearchSurface( cloud );
	AGASTDetector->setInputCloud( cloud );

	AGASTDetector->compute( *keyPoints);
	INFO(logger, "Detected " << keyPoints->size() << " keypoints");
	TRACE(logger, "detect: Finished");
	if(keyPoints->size() == 0) {

		std::runtime_error e("Could not find any keypoints");
		ERROR(logger, "detect: " << e.what() );
		throw e;
	}
	return keyPoints;
}

ScaleVec AGASTDetector::detect(const ColorVec &clouds) {

	TRACE(logger, "detect: Starting batch processing");
	ScaleVec vec;
	int i = 0;
	try {
		for(const auto &cloud : clouds) {
			i++;
			vec.emplace_back(detect( cloud ) );
		}
	}
	catch (std::runtime_error &e) {

		std::string tmp = e.what();
		std::runtime_error newE(tmp + " at image " + std::to_string(i));
		ERROR(logger, "detect: batch: " << newE.what());
		throw newE;
	}

	TRACE(logger, "detect: Finished batch processing");
	return vec;
}



} /* namespace Tagger3D */
