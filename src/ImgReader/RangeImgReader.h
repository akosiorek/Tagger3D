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
	ColorVec readImgs();

	int readLabel();

	virtual void setMode(int mode);

protected:
	void readImg(const std::string &colorPath, const std::string &depthPath, cv::Mat &colorImg, cv::Mat &depthImg);
	ColorCloud::Ptr readImg(const std::string &colorPath, const std::string &depthPath);

private:
	int count;
	int chunkSize;
	float depthScaleFactor;
	int resize;

	const std::string chunkSizeKey = moduleName + "chunkSize";
	const std::string depthScaleFactorKey = moduleName + "depthScaleFactor";
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


};

} /* namespace Tagger3D */
#endif /* RANGEIMGRADER_H_ */
