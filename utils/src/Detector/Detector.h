/*
 * Tagger3D : Detector.h
 *
 *  Created on: 	24 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include "../Common/clouds.h"
#include "../Common/ProcessObject.h"

namespace Tagger3D {

class Detector : public ProcessObject {
public:
	Detector() = delete;
	Detector(const std::map<std::string, std::string> &configMap);
	virtual ~Detector();

	virtual ScaleCloud::Ptr detect(const ColorCloud::Ptr &cloud) = 0;
	virtual ScaleVec detect(const ColorVec &clouds);

protected:
	const std::string moduleName = "detector" + separator;

private:
	const std::string loggerName = "Main.Detector";
};

} /* namespace Tagger3D */
#endif /* DETECTOR_H_ */
