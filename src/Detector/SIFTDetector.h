/*
 * Tagger3D : SIFTDetector.h
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef SIFTDETECTOR_H_
#define SIFTDETECTOR_H_

#include "Detector.h"
#include <pcl/keypoints/sift_keypoint.h>

namespace Tagger3D {

class SIFTDetector : public Detector {
public:
	SIFTDetector(const std::map<std::string, std::string> &configMap);
	virtual ~SIFTDetector();

	ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud);
	ScaleVec detect(const ColorVec &clouds);

private:
	SIFTDetector();
	bool createSiftDetector();

	std::unique_ptr<pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale>> siftDetector;

	float minScale;
	int octaves;
	int scalesPerOctave;
	float minContrast;

	const std::string minScaleKey = moduleName + "minScale";
	const std::string octavesKey = moduleName + "octaves";
	const std::string scalesPerOctaveKey = moduleName + "scalesPerOctave";
	const std::string minContrastKey = moduleName + "minContrast";


};

} /* namespace Tagger3D */
#endif /* SIFTDETECTOR_H_ */
