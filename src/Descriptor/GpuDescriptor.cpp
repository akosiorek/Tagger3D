/*
 * GpuDescriptor.cpp
 *
 *   Created on: 20 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "GpuDescriptor.h"
#include <pcl/gpu/features/features.hpp>

namespace Tagger3D {

GpuDescriptor::GpuDescriptor(const std::map<std::string, std::string> &configMap) : Descriptor(configMap) {}

GpuDescriptor::~GpuDescriptor() {}

cv::Mat GpuDescriptor::describe(const ColorCloud::Ptr &cloud,
		const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud ) {

	pcl::gpu::DeviceArray<pcl::PointXYZ> points;
	pcl::gpu::DeviceArray<pcl::PointXYZ> keypoints;
	pcl::gpu::DeviceArray<pcl::PointXYZ> normals;

	points.upload(xyzrgb2xyz(cloud->points));
	keypoints.upload(scale2xyz(keyCloud->points));
	normals.upload(normal2xyz(normalCloud->points));

	return describe(points, keypoints, normals);
}

std::vector<pcl::PointXYZ> GpuDescriptor::xyzrgb2xyz
	(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &vec) {

	std::vector<pcl::PointXYZ> xyz;
	xyz.reserve(vec.size());
	for(const auto &point : vec) {

		pcl::PointXYZ* p = new pcl::PointXYZ;
		p->x = point.x;
		p->y = point.y;
		p->z = point.z;
	    int r = std::max(1, std::min(255, static_cast<int>(point.r)));
		int g = std::max(1, std::min(255, static_cast<int>(point.g)));
		int b = std::max(1, std::min(255, static_cast<int>(point.b)));
		p->data[3] = (b << 16) + (g << 8) + r;
		xyz.push_back(*p);
	}
	return xyz;

}

std::vector<pcl::PointXYZ> GpuDescriptor::scale2xyz
	(std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale>> &vec) {

	std::vector<pcl::PointXYZ> xyz;
	xyz.reserve(vec.size());
	for(const auto &point : vec) {

		pcl::PointXYZ* p = new pcl::PointXYZ;
		memcpy((void*)p->data, (void*)point.data, 4 * sizeof(float));
		xyz.push_back(*p);
	}
	return xyz;
}

std::vector<pcl::PointXYZ> GpuDescriptor::normal2xyz
	(std::vector<pcl::Normal, Eigen::aligned_allocator<pcl::Normal>> &vec) {
	std::vector<pcl::PointXYZ> xyz;
	xyz.reserve(vec.size());
	for(const auto &point : vec)
		xyz.emplace_back(point.normal_x, point.normal_y, point.normal_z);

	return xyz;
}

} /* namespace Tagger3D */
