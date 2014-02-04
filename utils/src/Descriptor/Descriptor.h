/*
 * Tagger3D : Descriptor.h
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef DESCRIPTOR_H_
#define DESCRIPTOR_H_

#include "../Common/clouds.h"
#include "../Common/ProcessObject.h"

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace Tagger3D {



class Descriptor : public ProcessObject {
public:
	Descriptor(const std::map<std::string, std::string> &configMap);
	virtual ~Descriptor();

	virtual cv::Mat describe(const ColorCloud::Ptr &cloud, const ScaleCloud::Ptr &keyCloud, const NormalCloud::Ptr &normalCloud ) = 0;
	virtual std::vector<cv::Mat> describe(const ColorVec &clouds, const ScaleVec &keyClouds, const NormalVec &normalClouds);

protected:

	const std::string moduleName = "descriptor" + separator;
private:
	Descriptor();
	const std::string loggerName = "Main.Descriptor";

};

} /* namespace Tagger3D */
#endif /* DESCRIPTOR_H_ */
