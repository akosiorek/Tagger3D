/*
 * DenseDetector.cpp
 *
 *  Created on: 16 gru 2013
 *      Author: adam
 */

#include <DenseDetector.h>

namespace Tagger3D {

DenseDetector::DenseDetector(const std::map<std::string, std::string> &_configMap)
	: Detector(_configMap) {

	detector = std::unique_ptr<detectorType>(new detectorType());
	detector->setRadiusSearch(getParam<float>(radiusSearch));
}

ScaleCloud::Ptr DenseDetector::detect(const ColorCloud::Ptr &cloud) {
	TRACE(logger, "detect: Starting")

	pcl::PointCloud<int> sampled_indices;
	ScaleCloud::Ptr destCloud = ScaleCloud::Ptr(new ScaleCloud());

	detector->setInputCloud(cloud);
	detector->compute (sampled_indices);
	pcl::copyPointCloud (*cloud, sampled_indices.points, *destCloud);

	TRACE(logger, "detect: Finished")
	return destCloud;
}

} /* namespace Tagger3D */
