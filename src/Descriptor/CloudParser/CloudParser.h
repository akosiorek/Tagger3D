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
#include "PfhTraits.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace Tagger3D {

class CloudParser {

public:
	CloudParser();
	virtual ~CloudParser();

	template<typename T>
	static cv::Mat parse(const typename T::Ptr &cloud);

	template<typename T>
	static std::vector<cv::Mat> parse(const std::vector<typename T::Ptr> &cloud);

};

template<typename T>
inline cv::Mat CloudParser::parse(const typename T::Ptr& cloud) {

	int size = cloud->size();
	int matSize = PfhTraits<typename T::PointType>::size();
	cv::Mat mat( size, matSize, CV_32FC1 );
	for(int i = 0; i < size; i++) {
		float* dPtr = mat.ptr<float>(i);
		float* hPtr = cloud->points[i].histogram;
		for(int j = 0; j < matSize; j++)
			dPtr[j] = hPtr[j];
	}
	return mat;
}

template<typename T>
inline std::vector<cv::Mat> CloudParser::parse(
		const std::vector<typename T::Ptr>& clouds) {

	std::vector<cv::Mat> vec;
	for(const auto &cloud : clouds) {

		vec.push_back( parse<T>( cloud ) );
	}
	return vec;
}

} /* namespace Tagger3D */
#endif /* CLOUDPARSER_H_ */
