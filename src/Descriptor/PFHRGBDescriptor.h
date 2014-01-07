/*
 * PFHRGBDescriptor.h
 *
 *   Created on: 11 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef PFHRGBDESCRIPTOR_H_
#define PFHRGBDESCRIPTOR_H_

#include "Descriptor.h"

#include <pcl/features/pfhrgb.h>

namespace Tagger3D {

/*
 *
 */
class PFHRGBDescriptor: public Descriptor {
	/**
	 *
	 */
public:
	PFHRGBDescriptor() = delete;
	PFHRGBDescriptor(const std::map<std::string, std::string> &configMap);
	virtual ~PFHRGBDescriptor() = default;

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:

	void createDescriptor();
	typedef pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> descriptorType;
	std::unique_ptr<descriptorType> descriptor;

	//	Config keys
	const std::string radiusSearch = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */
#endif /* PFHRGBDESCRIPTOR_H_ */
