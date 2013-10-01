/*
 * Tagger3D : PointNormal.cpp
 *
 *  Created on: 	28 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "PointNormal.h"

namespace Tagger3D {

PointNormal::PointNormal(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap){

	logger = lgr::Logger::getLogger( loggerName );
	DEBUG(logger, "Creating PointNormal");
}

PointNormal::~PointNormal() {

	DEBUG(logger, "Destroying PointNormal");
}

void PointNormal::cleanupInputCloud( ColorCloud::Ptr &cloud) {

	TRACE(logger, "cleanupInputCloud: Starting");
	size_t j = 0;
	size_t indexSize = index.size();
	size_t cloudSize = cloud->points.size();
	if( cloudSize == indexSize ) {

		return;
	}

	for(size_t i = 0; i < indexSize; ++i) {

		cloud->points[i] = cloud->points[index[i]];
	}

	cloud->resize(indexSize);
	cloud->height = 1;
	cloud->width = indexSize;
	TRACE(logger, "cleanupInputCloud: Finished");
}

} /* namespace Tagger3D */
