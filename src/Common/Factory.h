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

	enum Dataset : unsigned char { B3DO, TOKYO };
	enum DetectorType : unsigned char { SIFT, ISS3D };
	enum DescriptorType : unsigned char { PFH, FPFH, PFHRGB };
	enum PredictorType : unsigned char { SVM };
};

} // namespace Tagger3D

#endif /* FACTORY_H_ */
