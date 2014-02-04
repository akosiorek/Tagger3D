/*
 * ShotColorDescriptor.h
 *
 *  Created on: 6 sty 2014
 *      Author: adam
 */

#ifndef SHOTCOLORDESCRIPTOR_H_
#define SHOTCOLORDESCRIPTOR_H_

#include <Descriptor.h>

#include <pcl/features/shot_omp.h>

namespace Tagger3D {

class ShotColorDescriptor: public Descriptor {
public:
	ShotColorDescriptor() = delete;
	ShotColorDescriptor(const std::map<std::string, std::string> &configMap);
	virtual ~ShotColorDescriptor() = default;

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:

	void createDescriptor();
	typedef pcl::SHOT1344 PointType;
	typedef pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointType> descriptorType;
	std::unique_ptr<descriptorType> descriptor;

	//	Config keys
	const std::string supportRadius = moduleName + "supportRadius";
};

} /* namespace Tagger3D */

#endif /* SHOTCOLORDESCRIPTOR_H_ */
