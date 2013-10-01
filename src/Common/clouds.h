/*
 * Tagger3D : clouds.h
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef CLOUDS_H_
#define CLOUDS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

/**
 *	Cloud types
 */
typedef pcl::PointCloud<pcl::PointWithScale> ScaleCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
typedef pcl::PointCloud<pcl::PFHSignature125> PfhCloud;
typedef pcl::PointCloud<pcl::FPFHSignature33> FpfhCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/**
 * Cloud vectors
 */
typedef std::vector<ScaleCloud::Ptr> ScaleVec;
typedef std::vector<ColorCloud::Ptr> ColorVec;
typedef std::vector<PfhCloud::Ptr> PfhVec;
typedef std::vector<FpfhCloud::Ptr> FpfhVec;
typedef std::vector<NormalCloud::Ptr> NormalVec;

#endif /* CLOUDS_H_ */
