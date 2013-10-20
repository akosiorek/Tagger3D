/*
 * PFHRGBgpu.cpp
 *
 *   Created on: 20 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "PFHRGBgpu.h"
#include "CloudParser/CloudParser.h"

namespace Tagger3D {

PFHRGBgpu::PFHRGBgpu(const std::map<std::string, std::string> &configMap) : GpuDescriptor(configMap) {

	createDescriptor();
	assert(descriptor != nullptr);}

PFHRGBgpu::~PFHRGBgpu() {}

void PFHRGBgpu::createDescriptor() {

	TRACE(logger, "creating gpu::PFHRB descriptor");
	descriptor = std::unique_ptr<descriptorType>(new descriptorType()) ;
	descriptor->setRadiusSearch(getParam<float>(radiusSearch), 0);
}

cv::Mat PFHRGBgpu::describe(const pcl::gpu::DeviceArray<pcl::PointXYZ> &cloud,
				const pcl::gpu::DeviceArray<pcl::PointXYZ> &keyCloud,const pcl::gpu::DeviceArray<pcl::PointXYZ> &normalCloud) {

	TRACE(logger, "describe PFHRGB: Starting");
	pcl::gpu::DeviceArray2D<pcl::PFHRGBSignature250> tempDescriptors;
	descriptor->setInputNormals( normalCloud );
	descriptor->setSearchSurface( cloud );
	descriptor->setInputCloud( keyCloud );
	descriptor->compute( tempDescriptors );
	TRACE(logger, "describe PFHRGB: Finished; descriptors size = " << tempDescriptors.rows() << "x" << tempDescriptors.cols());
	PfhRgbCloud::Ptr c(new PfhRgbCloud());
	int rows = tempDescriptors.rows();
	int cols = tempDescriptors.cols();
	for(int i = 0; i < rows; i++) {
		pcl::PFHRGBSignature250 ptr = tempDescriptors.ptr(i)[0];
		c->push_back(ptr);
//		for(int j = 0; j < cols; j++) {
//			TRACE(logger, "i = " << i << " j = " << j);
//			c->push_back(ptr);
//			TRACE(logger, "one point");
//		}
	}
	TRACE(logger, "done");
	return CloudParser::parse<PfhRgbCloud>(c);
}

} /* namespace Tagger3D */
