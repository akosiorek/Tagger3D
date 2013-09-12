/*
 * Iss3dDetector.cpp
 *
 *   Created on: 8 wrz 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "Iss3dDetector.h"

#include <pcl/search/kdtree.h>
#include <assert.h>

namespace Tagger3D {

Iss3dDetector::Iss3dDetector(const std::map<std::string, std::string> &configMap) : Detector(configMap) {


	createDetector();
	assert(detector != nullptr);

}

Iss3dDetector::~Iss3dDetector() {}

void Iss3dDetector::createDetector() {

	std::unique_ptr<pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB>> temp(new pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	float resolution = getParam<float>(modelResolution);
	salientRadius = 6 * resolution;
	nonMaxRadius = 4 * resolution;
	normalRadius = 4 * resolution;
	borderRadius = 1 * resolution;

	temp->setSearchMethod(tree);
	temp->setSalientRadius(salientRadius);
	temp->setNonMaxRadius(nonMaxRadius);
	temp->setBorderRadius(borderRadius);
	temp->setNormalRadius(normalRadius);
	temp->setThreshold21(getParam<float>(gamma21));
	temp->setThreshold32(getParam<float>(gamma32));
	temp->setMinNeighbors(getParam<int>(minNeighbours));
	temp->setNumberOfThreads(getParam<int>(threads));
	detector = std::move(temp);
}

ScaleCloud::Ptr Iss3dDetector::detect(const ColorCloud::Ptr &cloud) {

	TRACE(logger, "detect: Starting");
	ColorCloud::Ptr keyPoints(new ColorCloud() );
	//detector->setSearchSurface( cloud );
	detector->setInputCloud( cloud );

	detector->compute( *keyPoints);
	DEBUG(logger, "Detected " << keyPoints->size() << " keypoints");
	TRACE(logger, "detect: Finished");
	if(keyPoints->size() == 0) {

		std::runtime_error e("Could not find any keypoints");
		ERROR(logger, "detect: " << e.what() );
		throw e;
	}

	ScaleCloud::Ptr keyPointsScale(new ScaleCloud());
	pcl::copyPointCloud(*keyPoints, *keyPointsScale);
	return keyPointsScale;
}

} /* namespace Tagger3D */
