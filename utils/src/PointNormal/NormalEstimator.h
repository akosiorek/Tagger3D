/*
 * Tagger3D : NormalEstimator.h
 *
 *  Created on: 	28 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef NORMALESTIMATOR_H_
#define NORMALESTIMATOR_H_

#include "PointNormal.h"

#include <pcl/features/normal_3d_omp.h>

namespace Tagger3D {

class NormalEstimator: public PointNormal {
public:
	NormalEstimator() = delete;
	NormalEstimator(const std::map<std::string, std::string> &configMap);
	virtual ~NormalEstimator() = default;

	NormalCloud::Ptr computeNormals(const ColorCloud::Ptr &cloud);
	NormalVec computeNormals(const ColorVec &clouds);


private:
	void createNormalEstimator();

	std::unique_ptr<pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal>> normalEstimator;

	//	Config keys
	const std::string normalRadius = moduleName + "normalRadius";
	const std::string kNN = moduleName + "kNN";
};

} /* namespace Tagger3D */
#endif /* NORMALESTIMATOR_H_ */
