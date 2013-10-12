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
	PFHRGBDescriptor(const std::map<std::string, std::string> &configM);
	virtual ~PFHRGBDescriptor();

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:
	PFHRGBDescriptor();
	void createDescriptor();
	typedef pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> descriptorType;
	std::unique_ptr<descriptorType> descriptor;

	//	Config parameters
	float radiusSearch;

	//	Config keys
	const std::string radiusSearchKey = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */
#endif /* PFHRGBDESCRIPTOR_H_ */
