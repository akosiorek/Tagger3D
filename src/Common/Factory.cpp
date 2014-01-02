/*
 * Factory.cpp
 *
 *  Created on: 14 gru 2013
 *      Author: adam
 */

#include "Factory.h"

#include "RangeImgReader.h"
#include "PcdReader.h"
#include "NormalEstimator.h"
#include "Detector.h"
#include "SIFTDetector.h"
#include "Iss3dDetector.h"
#include "DenseDetector.h"

#include "PFHDescriptor.h"
#include "FPFHDescriptor.h"
#include "PFHRGBDescriptor.h"
#include "KMeansCluster.h"
#include "SVMPredictor.h"

#include <stdexcept>

namespace Tagger3D {

Factory::Factory(const std::map<std::string, std::string> &_configMap)
: ProcessObject(_configMap),
  configMap(_configMap) {

	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Tagger3D");
}

std::unique_ptr<ImgReader> Factory::getReader() const {

	std::unique_ptr<ImgReader> imgReader;
	switch(getParam<int>(readerType)) {
	case Dataset::TOKYO: imgReader = std::unique_ptr<ImgReader>(new RangeImgReader(configMap)); break;
	case Dataset::B3DO: imgReader = std::unique_ptr<ImgReader>(new PcdReader(configMap)); break;
	default:
		std::runtime_error e("Invalid reader type");
		ERROR(logger, e.what());
		throw e;
	}
	return imgReader;
}

std::unique_ptr<PointNormal> Factory::getPointNormal() const {

	return std::unique_ptr<PointNormal> (new NormalEstimator(configMap));
}

std::unique_ptr<Detector> Factory::getDetector() const {

	std::unique_ptr<Detector> detector;
	switch( getParam<int>(detectorType)) {
	case DetectorType::SIFT: detector = std::unique_ptr<Detector> (new SIFTDetector(configMap)); break;
	case DetectorType::ISS3D: detector = std::unique_ptr<Detector> (new Iss3dDetector(configMap)); break;
	case DetectorType::DENSE: detector = std::unique_ptr<Detector> (new DenseDetector(configMap)); break;
	default:
			std::runtime_error e("Invalid detector type");
			ERROR(logger, e.what());
			throw e;
		}
	return detector;
}

std::unique_ptr<Descriptor> Factory::getDescriptor() const {
	std::unique_ptr<Descriptor> descriptor;
	switch( getParam<int>( descType )) {
	case DescriptorType::PFH: descriptor = std::unique_ptr<Descriptor> (new PFHDescriptor(configMap)); break;
	case DescriptorType::FPFH: descriptor = std::unique_ptr<Descriptor> (new FPFHDescriptor(configMap)); break;
	case DescriptorType::PFHRGB: descriptor = std::unique_ptr<Descriptor> (new PFHRGBDescriptor(configMap)); break;
//	case DescriptorType::PFHRGB: descriptor = std::unique_ptr<Descriptor> (new PFHRGBgpu(configMap)); break;
	default:
		std::runtime_error e("Invalid descriptor type");
		ERROR(logger, e.what());
		throw e;
	}
	return descriptor;
}

std::unique_ptr<Cluster> Factory::getCluster() const {

	return std::unique_ptr<Cluster> (new KMeansCluster(configMap));
}

std::unique_ptr<Predictor> Factory::getPredictor() const {

	return std::unique_ptr<Predictor> (new SVMPredictor(configMap));
}

} //namespace Tagger3D

