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
	: ProcessObject(_configMap),
	  loaded(false) {

	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Cluster");

	clusterCount = getParam<int>(clusterCountKey);
	dimCount = getParam<int>(dimCountKey);
}

Cluster::~Cluster() {

	DEBUG(logger, "Destroying Cluster");
}

cv::Mat Cluster::cluster(const std::vector<cv::Mat> &descriptors) {

	const cv::Mat *dPtr = &descriptors[0];
	long size = descriptors.size();

	cv::Mat clustered = cluster(dPtr[0]);
	clustered.reserve(size);
	for(int i = 1; i < size; ++i) {
		clustered.push_back(cluster(dPtr[i]));
	}

	return clustered.clone();
}


} /* namespace Tagger3D */
