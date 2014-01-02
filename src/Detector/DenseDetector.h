/*
 * DenseDetector.h
 *
 *  Created on: 16 gru 2013
 *      Author: adam
 */

#ifndef DENSEDETECTOR_H_
#define DENSEDETECTOR_H_

#include <Detector.h>

#include <pcl/keypoints/uniform_sampling.h>

namespace Tagger3D {

class DenseDetector: public Detector {
public:
	DenseDetector() = delete;
	DenseDetector(const std::map<std::string, std::string> &_configMap);
	virtual ~DenseDetector() = default;

	ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud);

	typedef pcl::UniformSampling<pcl::PointXYZRGB> detectorType;
	typedef std::unique_ptr<detectorType> detectorPtrType;

private:

	detectorPtrType detector;
	const std::string radiusSearch = moduleName + "radiusSearch";
};

} /* namespace Tagger3D */

#endif /* DENSEDETECTOR_H_ */
