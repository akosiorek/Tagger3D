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

} /* namespace Tagger3D */
