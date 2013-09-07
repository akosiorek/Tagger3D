/*
 * Tagger3D : SIFTDetector.cpp
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "SIFTDetector.h"

#include <pcl/search/kdtree.h>
#include <assert.h>

namespace Tagger3D {

SIFTDetector::SIFTDetector(const std::map<std::string, std::string> &configMap) : Detector(configMap) {

	minScale = getParam<float>(minScaleKey);
	octaves = getParam<int>(octavesKey);
	scalesPerOctave = getParam<int>(scalesPerOctaveKey);
	minContrast = getParam<float>(minContrastKey);
	createSiftDetector();
	assert( siftDetector != nullptr );
}

SIFTDetector::~SIFTDetector() {}

bool SIFTDetector::createSiftDetector() {

	TRACE(logger, "createSiftDetector: Starting");
	decltype(siftDetector) detector( new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	detector->setSearchMethod(kdTree);
	detector->setScales(minScale, octaves, scalesPerOctave);
	detector->setMinimumContrast(minContrast);

	siftDetector = std::move( detector );

	TRACE(logger, "createSiftDetector: Finished");
	return true;
}

ScaleCloud::Ptr SIFTDetector::detect(const ColorCloud::Ptr &cloud) {

	TRACE(logger, "detect: Starting");
	ScaleCloud::Ptr keyPoints(new ScaleCloud() );
	siftDetector->setSearchSurface( cloud );
	siftDetector->setInputCloud( cloud );

	siftDetector->compute( *keyPoints);
	DEBUG(logger, "Detected " << keyPoints->size() << " keypoints");
	TRACE(logger, "detect: Finished");
	if(keyPoints->size() == 0) {

		std::runtime_error e("Could not find any keypoints");
		ERROR(logger, "detect: " << e.what() );
		throw e;
	}

//	for( auto &point : keyPoints->points)
//		std::cout << point.x << "," << point.y << "," << point.z << " ";
//	std::cout << std::endl;
	return keyPoints;
}

ScaleVec SIFTDetector::detect(const ColorVec &clouds) {

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