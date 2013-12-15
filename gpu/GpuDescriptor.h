/*
 * GpuDescriptor.h
 *
 *   Created on: 20 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef GPUDESCRIPTOR_H_
#define GPUDESCRIPTOR_H_

#include "Descriptor.h"
#include <pcl/gpu/features/features.hpp>

namespace Tagger3D {

/*
 *
 */
class GpuDescriptor: public Descriptor {
	/**
	 *
	 */
public:
	GpuDescriptor(const std::map<std::string, std::string> &configMap);
	virtual ~GpuDescriptor();

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

protected:
	virtual cv::Mat describe(const pcl::gpu::DeviceArray<pcl::PointXYZ> &cloud,
			const pcl::gpu::DeviceArray<pcl::PointXYZ> &keycloud,const pcl::gpu::DeviceArray<pcl::PointXYZ> &normalCloud) = 0;

private:
	GpuDescriptor();

	std::vector<pcl::PointXYZ> xyzrgb2xyz(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &vec);
	std::vector<pcl::PointXYZ> scale2xyz(std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale>> &vec);
	std::vector<pcl::PointXYZ> normal2xyz(std::vector<pcl::Normal, Eigen::aligned_allocator<pcl::Normal>> &vec);
};

} /* namespace Tagger3D */
#endif /* GPUDESCRIPTOR_H_ */
