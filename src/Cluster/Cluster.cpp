/*
 * Cluster.cpp
 *
 *  Created on: 	20-06-2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#include "Cluster.h"

#include <stdexcept>

namespace Tagger3D {

Cluster::Cluster(const std::map<std::string, std::string>& _configMap)
	: ProcessObject(_configMap) {


	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Cluster");
	loaded = false;
}

Cluster::~Cluster() {

	DEBUG(logger, "Destroying Cluster");
}


} /* namespace Tagger3D */
