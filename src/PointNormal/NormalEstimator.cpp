/*
 * Tagger3D : NormalEstimator.cpp
 *
 *  Created on: 	28 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "NormalEstimator.h"

#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <assert.h>

namespace Tagger3D {

NormalEstimator::NormalEstimator(const std::map<std::string, std::string> &configMap) : PointNormal(configMap) {

	normalRadius = getParam<float>( normalRadiusKey );
	normalEstimator = nullptr;
	assert( normalEstimator == nullptr );
	createNormalEstimator();
	assert( normalEstimator != nullptr );
}

NormalEstimator::~NormalEstimator() {}

void NormalEstimator::createNormalEstimator() {

	TRACE(logger, "createNormalEstimator: Starting");
	std::unique_ptr<pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>> estimator(new pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	assert( kdTree != nullptr );
	assert( normalRadius > 0);
	estimator->setSearchMethod( kdTree );
	estimator->setRadiusSearch( normalRadius );
	//estimator->setKSearch(5);
	TRACE(logger, "Normal Radius = " << normalRadius );
	normalEstimator = std::move( estimator );
	TRACE(logger, "createNormalEstimator: Finished");
}

NormalCloud::Ptr NormalEstimator::computeNormals(const ColorCloud::Ptr &cloud) {

	TRACE(logger, "computeNormals: Starting");

	NormalCloud::Ptr normalCloud( new NormalCloud() );
	normalEstimator->setInputCloud( cloud );
	normalEstimator->compute( *normalCloud );
	pcl::removeNaNNormalsFromPointCloud( *normalCloud, *normalCloud, index);

	TRACE(logger, "computeNormals: Finished");
	return normalCloud;
}

NormalVec NormalEstimator::computeNormals(const ColorVec &clouds) {

	TRACE(logger, "computeNormals: Starting batch processing");
	NormalVec vec;
	for(auto &cloud : clouds) {

		vec.emplace_back( computeNormals( cloud ) );
	}

	TRACE(logger, "computeNormals: Finished batch processing");
	return vec;
}

} /* namespace Tagger3D */
