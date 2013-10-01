/*
 * Tagger3D : CloudParser.h
 *
 *  Created on: 	31 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	Class for parsing different types of point clouds to cv::Mat
 */

#ifndef CLOUDPARSER_H_
#define CLOUDPARSER_H_

#include "../../Common/logger.h"
#include "../../Common/clouds.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace Tagger3D {

class CloudParser {

public:
	CloudParser();
	virtual ~CloudParser();

	cv::Mat parsePFH(const PfhCloud::Ptr &cloud);
	std::vector<cv::Mat> parsePFH(const PfhVec &clouds);

	cv::Mat parseFPFH(const FpfhCloud::Ptr &cloud);
	std::vector<cv::Mat> parseFPFH(const FpfhVec &clouds);

private:

	static const int PFH125 = 125;
	static const int FPFH33 = 33;

	const std::string loggerName = "Main.CloudParser";
	lgr::LoggerPtr logger = lgr::Logger::getLogger( loggerName );
};

} /* namespace Tagger3D */
#endif /* CLOUDPARSER_H_ */
