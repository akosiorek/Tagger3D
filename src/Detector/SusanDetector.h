/*
 * SusanDetector.h
 *
 *   Created on: 8 wrz 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef SUSANDETECTOR_H_
#define SUSANDETECTOR_H_

#include "Detector.h"

#include <pcl/keypoints/susan.h>

namespace Tagger3D {

/*
 *
 */
class SusanDetector: public Tagger3D::Detector {
	/**
	 *
	 */
public:
	SusanDetector(const std::map<std::string, std::string> &configMap);
	virtual ~SusanDetector();

	//ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud);

private:
	SusanDetector();
};

} /* namespace Tagger3D */
#endif /* SUSANDETECTOR_H_ */
