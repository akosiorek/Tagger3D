/*
 * FPFHDescriptor.h
 *
 *   Created on: 30 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef FPFHDESCRIPTOR_H_
#define FPFHDESCRIPTOR_H_

#include "Descriptor.h"

#include <pcl/features/fpfh_omp.h>

namespace Tagger3D {

/*
 *
 */
class FPFHDescriptor: public Descriptor {
	/**
	 *
	 */
public:
	FPFHDescriptor(const std::map<std::string, std::string> &configM);
	virtual ~FPFHDescriptor();

	cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud );

private:
	FPFHDescriptor();
	void createFpfhDescriptor();

	std::unique_ptr<pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>> descriptor;

	//	Config parameters
	float radiusSearch;

	//	Config keys
	const std::string radiusSearchKey = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */
#endif /* FPFHDESCRIPTOR_H_ */
