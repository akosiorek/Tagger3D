/*
 * ShotDescriptor.cpp
 *
 *  Created on: 6 sty 2014
 *      Author: adam
 */

#include <ShotDescriptor.h>
#include "CloudParser.h"

namespace Tagger3D {

ShotDescriptor::ShotDescriptor(const std::map<std::string, std::string> &configMap) : Descriptor(configMap) {

	createDescriptor();
	assert(descriptor != nullptr);
}

cv::Mat ShotDescriptor::describe(const ColorCloud::Ptr& cloud,
		const ScaleCloud::Ptr& keyCloud, const NormalCloud::Ptr& normalCloud) {

	TRACE(logger, "describe: Starting");
	ColorCloud::Ptr keyCloudRgb( new ColorCloud() );
	pcl::copyPointCloud( *keyCloud, *keyCloudRgb);

	pcl::PointCloud<PointType>::Ptr descriptors (new pcl::PointCloud<PointType>());
	descriptor->setInputCloud(keyCloudRgb);
	descriptor->setInputNormals(normalCloud);
	descriptor->setSearchSurface(cloud);
	descriptor->compute(*descriptors);

	TRACE(logger, "describe: Finished");
	return CloudParser::parse<pcl::PointCloud<PointType>>(descriptors);
}

void ShotDescriptor::createDescriptor() {

	TRACE(logger, "createDescriptor: Starting");
	decltype(descriptor) newDescriptor( new descriptorType() );
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree( new pcl::search::KdTree<pcl::PointXYZRGB>() );
	newDescriptor->setSearchMethod( kdTree );
	newDescriptor->setRadiusSearch(getParam<float>(supportRadius));

	descriptor = std::move( newDescriptor );
	TRACE(logger, "createDescriptor: Finished");
}

} /* namespace Tagger3D */
