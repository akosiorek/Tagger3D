/*
 * ShotDescriptor.h
 *
 *  Created on: 6 sty 2014
 *      Author: adam
 */

#ifndef SHOTDESCRIPTOR_H_
#define SHOTDESCRIPTOR_H_

#include <Descriptor.h>

#include <pcl/features/shot_omp.h>

namespace Tagger3D {

class ShotDescriptor: public Descriptor {
public:
	ShotDescriptor() = delete;
	ShotDescriptor(const std::map<std::string, std::string> &configMap);
	virtual ~ShotDescriptor() = default;

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:

	void createDescriptor();
	typedef pcl::SHOT352 PointType;
	typedef pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointType> descriptorType;
	std::unique_ptr<descriptorType> descriptor;

	//	Config keys
	const std::string supportRadius = moduleName + "supportRadius";
};

} /* namespace Tagger3D */

#endif /* SHOTDESCRIPTOR_H_ */
