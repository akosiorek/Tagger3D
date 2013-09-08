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
	ScaleVec vec;
	int i = 0;
	try {
		for(const auto &cloud : clouds) {
			i++;
			vec.emplace_back(detect( cloud ) );
		}
	}
	catch (std::runtime_error &e) {

		std::string tmp = e.what();
		std::runtime_error newE(tmp + " at image " + std::to_string(i));
		ERROR(logger, "detect: batch: " << newE.what());
		throw newE;
	}

	TRACE(logger, "detect: Finished batch processing");
	return vec;
}

} /* namespace Tagger3D */
