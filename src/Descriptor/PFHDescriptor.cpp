/*
 * Tagger3D : PFHDescriptor.cpp
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "PFHDescriptor.h"

#include <pcl/search/kdtree.h>
#include <assert.h>

namespace Tagger3D {

PFHDescriptor::PFHDescriptor(const std::map<std::string, std::string> &configMap) : Descriptor(configMap) {

	radiusSearch = getParam<float>( radiusSearchKey );
	createPfhDescriptor();
	assert(descriptor != nullptr);
	assert(parser != nullptr);
}

PFHDescriptor::~PFHDescriptor() {}

void PFHDescriptor::createPfhDescriptor() {

	TRACE(logger, "createPfhDescriptor: Starting");
	decltype(descriptor) newDescriptor( new pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	newDescriptor->setSearchMethod( kdTree );
	newDescriptor->setRadiusSearch( radiusSearch );

	descriptor = std::move( newDescriptor );
	parser = std::unique_ptr<CloudParser>( new CloudParser() );
	TRACE(logger, "createPfhDescriptor: Finished");
}

cv::Mat PFHDescriptor::describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud) {

	TRACE(logger, "describe: Starting");
	PfhCloud::Ptr descriptors( new PfhCloud() );
	ColorCloud::Ptr keyCloudRgb( new ColorCloud() );
	pcl::copyPointCloud( *keyCloud, *keyCloudRgb);
	descriptor->setInputNormals( normalCloud );
	descriptor->setSearchSurface( cloud );
	descriptor->setInputCloud( keyCloudRgb );
	descriptor->compute( *descriptors );
	TRACE(logger, "describe: Finished");
	return parser->parsePFH(descriptors);
}



} /* namespace Tagger3D */
