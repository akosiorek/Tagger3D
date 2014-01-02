/*
 * Predictor.cpp
 *
 *  Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 * Description: Predictor base class implementation
 */

#include "Predictor.h"

namespace Tagger3D {

Predictor::Predictor(const std::map<std::string, std::string> &_configMap,
		const std::string predictorType)

	: ProcessObject(_configMap),
	  moduleName(predictorType + separator) {


	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Predictor");

	if( _configMap.empty() ) {

		throw std::invalid_argument("Empty configuration map");
	}
}

Predictor::~Predictor() {

	TRACE(logger, "Destroying Predictor");
}

} /* namespace semantic_tagger */
