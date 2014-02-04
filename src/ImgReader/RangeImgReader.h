/*
 * Tagger3D : RangeImgReader.h
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#ifndef RANGEIMGRADER_H_
#define RANGEIMGRADER_H_

#include "ImgReader.h"

#include <opencv2/core/core.hpp>

namespace Tagger3D {

class RangeImgReader: public ImgReader {
public:
	RangeImgReader(const std::map<std::string, std::string> &configMap);
	virtual ~RangeImgReader();

	ColorCloud::Ptr readImg();

	int readLabel();

	virtual void setMode(int mode);

protected:
	void readImg(const std::string &colorPath, const std::string &depthPath, cv::Mat &colorImg, cv::Mat &depthImg);
	ColorCloud::Ptr readImg(const std::string &colorPath, const std::string &depthPath);

private:
	int count;
	int resize;

	const std::string resizeKey = moduleName + "resize";
	const std::string trainColorImg = moduleName + "trainColorImgList";
	const std::string trainDepthImg = moduleName + "trainDepthImgList";
	const std::string trainLabel = moduleName + "trainLabelsList";
	const std::string testColorImg = moduleName + "testColorImgList";
	const std::string testDepthImg = moduleName + "testDepthImgList";
	const std::string testLabel = moduleName + "testLabelsList";

	std::vector<std::string> depthImgVec;
	std::vector<std::string> colorImgVec;


	ColorCloud::Ptr matToCloud(const cv::Mat &colorImg, const cv::Mat &depthImg);

	const float factorX0 =  320.0f * 3.501e-3f;
	const float factorY0 = 240.0f * 3.501e-3f;

	const float depthWidth = 320.0f;
	const float depthHeight = 240.0f;
	const float depthWidthHalf = depthWidth / 2.0f;
	const float depthHeightHalf = depthHeight / 2.0f;
	const float depthHFOV = 57.0f;
	const float depthVFOV = 43.0f;
	const float depthH = tan ( (depthHFOV / 2.0f) * ( M_PI / 180.0f ) );
	const float depthV = tan ( (depthVFOV / 2.0f) * ( M_PI / 180.0f ) );


};

} /* namespace Tagger3D */
#endif /* RANGEIMGRADER_H_ */
