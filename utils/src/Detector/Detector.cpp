/*
 * Tagger3D : Detector.cpp
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "Detector.h"

namespace Tagger3D {

Detector::Detector(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap) {

	logger = lgr::Logger::getLogger( loggerName );
	DEBUG(logger, "Creating Detector");
}

Detector::~Detector() {

	DEBUG(logger, "Destroying Detector");
}

ScaleVec Detector::detect(const ColorVec &clouds) {
	TRACE(logger, "detect: Starting batch processing");

	int size = clouds.size();
	ScaleVec vec;
	vec.reserve(size);
	auto cloudPtr = &clouds[0];

	for(int i = 0; i < size; ++i) {
		vec.push_back(std::move(detect(cloudPtr[i])));
	}

	TRACE(logger, "detect: Finished batch processing");
	return vec;
}

} /* namespace Tagger3D */
