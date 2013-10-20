/*
 * PFHRGBDescriptor.cpp
 *
 *   Created on: 11 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "PFHRGBDescriptor.h"
#include "CloudParser/CloudParser.h"

namespace Tagger3D {

PFHRGBDescriptor::PFHRGBDescriptor(const std::map<std::string, std::string> &configMap) : Descriptor(configMap) {
	createDescriptor();
	assert(descriptor != nullptr);
}

PFHRGBDescriptor::~PFHRGBDescriptor() {}

void PFHRGBDescriptor::createDescriptor() {

	TRACE(logger, "createDescriptor: Starting");
	decltype(descriptor) newDescriptor( new descriptorType() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	newDescriptor->setSearchMethod( kdTree );
	newDescriptor->setRadiusSearch(getParam<float>(radiusSearch));

	descriptor = std::move( newDescriptor );
	TRACE(logger, "createDescriptor: Finished");
}

cv::Mat PFHRGBDescriptor::describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud) {

	TRACE(logger, "describe PFHRGB: Starting");
	PfhRgbCloud::Ptr descriptors( new PfhRgbCloud() );
	ColorCloud::Ptr keyCloudRgb( new ColorCloud() );
	pcl::copyPointCloud( *keyCloud, *keyCloudRgb);
	descriptor->setInputNormals( normalCloud );
	descriptor->setSearchSurface( cloud );
	descriptor->setInputCloud( keyCloudRgb );
	descriptor->compute( *descriptors );
	TRACE(logger, "describe PFHRGB: Finished; descriptors size = " << descriptors->size());
	return CloudParser::parse<PfhRgbCloud>(descriptors);
}


} /* namespace Tagger3D */
