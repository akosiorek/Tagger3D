/*
 * Tagger3D : PointNormal.h
 *
 *  Created on: 	28 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef POINTNORMAL_H_
#define POINTNORMAL_H_

#include "../Common/clouds.h"
#include "../Common/ProcessObject.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Tagger3D {

class PointNormal: public Tagger3D::ProcessObject {
public:
	PointNormal(const std::map<std::string, std::string> &configMap);
	virtual ~PointNormal();

	virtual NormalCloud::Ptr computeNormals(const ColorCloud::Ptr &cloud) = 0;
	virtual NormalVec computeNormals(const ColorVec &clouds) = 0;
	void cleanupInputCloud(ColorCloud::Ptr &cloud);

protected:
	const std::string moduleName = "PointNormal" + separator;
	std::vector<int> index;

private:
	PointNormal();
	const std::string loggerName = "Main.PointNormal";
};

} /* namespace Tagger3D */
#endif /* POINTNORMAL_H_ */
