/*
 * PFHRGBgpu.h
 *
 *   Created on: 20 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef PFHRGBGPU_H_
#define PFHRGBGPU_H_

#include "GpuDescriptor.h"

namespace Tagger3D {

/*
 *
 */
class PFHRGBgpu: public GpuDescriptor {
	/**
	 *
	 */
public:
	PFHRGBgpu(const std::map<std::string, std::string> &configMap);
	virtual ~PFHRGBgpu();

	virtual cv::Mat describe(const pcl::gpu::DeviceArray<pcl::PointXYZ> &cloud,
				const pcl::gpu::DeviceArray<pcl::PointXYZ> &keycloud,const pcl::gpu::DeviceArray<pcl::PointXYZ> &normalCloud);

private:
	PFHRGBgpu();
	void createDescriptor();
	typedef pcl::gpu::PFHRGBEstimation descriptorType;
	std::unique_ptr<descriptorType> descriptor;

	//	Config keys
	const std::string radiusSearch = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */
#endif /* PFHRGBGPU_H_ */
