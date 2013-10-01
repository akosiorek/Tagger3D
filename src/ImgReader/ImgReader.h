/*
 * Tagger3D : ImgReader.h
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:
 */
#ifndef IMGREADER_H_
#define IMGREADER_H_

#include "../Common/clouds.h"
#include "../Common/ProcessObject.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace Tagger3D {

class ImgReader : public ProcessObject{
public:
	ImgReader(const std::map<std::string, std::string> &configMap);
	virtual ~ImgReader();

	/**
	 *	Reads a single image.
	 *	@return a range image.
	 */
	virtual ColorCloud::Ptr readImg() = 0;

	/**
	 *	Reads a batch of images or reads images continuously;
	 *	@return	a vector of range images.
	 */
	virtual ColorVec readImgs() = 0;


	/**
	 * Returns a label for a previously read image
	 */
	virtual int readLabel() = 0;

	/**
	 * Reads labels for a batch of images
	 * @return a vector of labels
	 */
	virtual std::vector<int> readLabels();

protected:
	std::string typeToStr(const int &type);
	std::vector<std::string> getLineList(const std::string &path);
	std::string moduleName = "ImgReader" + separator;
	std::vector<std::string> labelVec;

	enum { TRAIN, TEST };

private:
	const std::string loggerName = "Main.ImgReader";
};

} /* namespace Tagger3D */
#endif /* IMGREADER_H_ */
