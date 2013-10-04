/*
 * FPFHDescriptor.cpp
 *
 *   Created on: 30 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "FPFHDescriptor.h"

#include <assert.h>

namespace Tagger3D {

FPFHDescriptor::FPFHDescriptor(const std::map<std::string, std::string> &configMap) : Descriptor(configMap) {

	radiusSearch = getParam<float>( radiusSearchKey );
	createFpfhDescriptor();
	assert(descriptor != nullptr);
	assert(parser != nullptr);
}

FPFHDescriptor::~FPFHDescriptor() {}

void FPFHDescriptor::createFpfhDescriptor() {

	TRACE(logger, "createFpfhDescriptor: Starting");
	decltype(descriptor) newDescriptor( new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	newDescriptor->setSearchMethod( kdTree );
	newDescriptor->setRadiusSearch( radiusSearch );

	descriptor = std::move( newDescriptor );
	parser = std::unique_ptr<CloudParser>( new CloudParser() );
	TRACE(logger, "createFpfhDescriptor: Finished");
}

cv::Mat FPFHDescriptor::describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud) {

	TRACE(logger, "describe fpfh: Starting");
	FpfhCloud::Ptr descriptors( new FpfhCloud() );
	ColorCloud::Ptr keyCloudRgb( new ColorCloud() );
	pcl::copyPointCloud( *keyCloud, *keyCloudRgb);
	descriptor->setInputNormals( normalCloud );
	descriptor->setSearchSurface( cloud );
	descriptor->setInputCloud( keyCloudRgb );
	descriptor->compute( *descriptors );
	TRACE(logger, "describe fpfh: Finished; descriptors size = " << descriptors->size());
	return parser->parseFPFH(descriptors);
}



} /* namespace Tagger3D */
