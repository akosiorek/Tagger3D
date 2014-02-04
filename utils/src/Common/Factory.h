/*
 * Factory.h
 *
 *  Created on: 14 gru 2013
 *      Author: adam
 */

#ifndef FACTORY_H_
#define FACTORY_H_

#include "ProcessObject.h"

#include "ImgReader.h"
#include "PointNormal.h"
#include "Detector.h"
#include "Descriptor.h"
#include "Cluster.h"
#include "Predictor.h"

#include <memory>

namespace Tagger3D {

class Factory: public ProcessObject {
public:
	Factory() = delete;
	Factory(const std::map<std::string, std::string> &_configMap);
	virtual ~Factory() = default;

	std::unique_ptr<ImgReader> getReader() const;
	std::unique_ptr<PointNormal> getPointNormal() const;
	std::unique_ptr<Detector> getDetector() const;
	std::unique_ptr<Descriptor> getDescriptor() const;
	std::unique_ptr<Cluster> getCluster() const;
	std::unique_ptr<Predictor> getPredictor() const;

private:

	const std::map<std::string, std::string> &configMap;

	const std::string readerType = "readerType";
	const std::string detectorType = "detectorType";
	const std::string descType = "descType";
	const std::string predictorType = "predictorType";

	const std::string loggerName = "Main.Factory";

	enum Dataset { B3DO, TOKYO };
	enum DetectorType { SIFT, ISS3D, DENSE };
	enum DescriptorType { PFH, FPFH, PFHRGB, SHOT, SHOTCOLOR };
	enum PredictorType { SVM_LIB, SVM_CV };
};

} // namespace Tagger3D

#endif /* FACTORY_H_ */
