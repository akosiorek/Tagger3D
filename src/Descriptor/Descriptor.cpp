/*
 * Tagger3D : Descriptor.cpp
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "Descriptor.h"

namespace Tagger3D {

Descriptor::Descriptor(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap) {

	logger = lgr::Logger::getLogger( loggerName );
	DEBUG(logger, "Creating Descriptor");
}

Descriptor::~Descriptor() {

	DEBUG(logger, "Destroying Descriptor");
}

std::vector<cv::Mat> Descriptor::describe(const ColorVec &clouds, const ScaleVec &keyClouds, const NormalVec &normalClouds) {

	TRACE(logger, "describe: Starting batch processing");
	if( clouds.size() != keyClouds.size() || clouds.size() != normalClouds.size() ) {

		throw std::invalid_argument("Clouds have different size");
	}
	std::vector<cv::Mat> vec;
	vec.reserve(clouds.size());
	auto keyCloud = keyClouds.begin();
	auto normalCloud = normalClouds.begin();
	for(const auto &cloud : clouds) {

		vec.push_back( describe(cloud, *keyCloud, *normalCloud) );
		++keyCloud;
		++normalCloud;
	}
	TRACE(logger, "describe: Finished batch processing");
	return vec;
}

} /* namespace Tagger3D */
