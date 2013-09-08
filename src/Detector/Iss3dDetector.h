/*
 * Iss3dDetector.h
 *
 *   Created on: 8 wrz 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef ISS3DDETECTOR_H_
#define ISS3DDETECTOR_H_

#include "Detector.h"

#include <pcl/keypoints/iss_3d.h>

namespace Tagger3D {

/*
 *
 */
class Iss3dDetector: public Tagger3D::Detector {
	/**
	 *
	 */
public:
	Iss3dDetector(const std::map<std::string, std::string> &configMap);
	virtual ~Iss3dDetector();

	ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud);

private:
	Iss3dDetector();
};

} /* namespace Tagger3D */
#endif /* ISS3DDETECTOR_H_ */
