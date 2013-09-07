/*
 * AGASTDetector.h
 *
 *   Created on: 28 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef AGASTDETECTOR_H_
#define AGASTDETECTOR_H_

#include "Detector.h"

#include <pcl/keypoints/agast_2d.h>

namespace Tagger3D {

/*
 *
 */
class AGASTDetector: public Detector {
	/**
	 *
	 */
public:
	AGASTDetector(const std::map<std::string, std::string> &configMap);
	virtual ~AGASTDetector();

	ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud);
	ScaleVec detect(const ColorVec &clouds);

private:
	AGASTDetector();
	bool createAgastDetector();

	std::unique_ptr<pcl::keypoints::agast::AbstractAgastDetector<pcl::PointXYZRGB, pcl::PointWithScale>> agastDetector;

	int height;
	int width;
	float threshold;
	float bmax;

	const std::string heightKey = moduleName + "height";
	const std::string widthKey = moduleName + "width";
	const std::string thresholdKey = moduleName + "threshold";
	const std::string bmaxKey = moduleName + "bmax";
};

} /* namespace Tagger3D */
#endif /* AGASTDETECTOR_H_ */
