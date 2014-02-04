/*
 * Tagger3D : PFHDescriptor.h
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef PFHDESCRIPTOR_H_
#define PFHDESCRIPTOR_H_

#include "Descriptor.h"

#include <pcl/features/pfh.h>


namespace Tagger3D {

class PFHDescriptor : public Descriptor {
public:
	PFHDescriptor(const std::map<std::string, std::string> &configMap);
	virtual ~PFHDescriptor();

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:
	PFHDescriptor();
	void createPfhDescriptor();

	std::unique_ptr<pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>> descriptor;

	//	Config parameters
	float radiusSearch;

	//	Config keys
	const std::string radiusSearchKey = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */
#endif /* PFHDESCRIPTOR_H_ */
