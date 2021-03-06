/*
 * Tagger3D : RangeImgReader.cpp
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#include "RangeImgReader.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <fstream>

namespace Tagger3D {

RangeImgReader::RangeImgReader(const std::map<std::string, std::string> &configMap) : ImgReader(configMap) {

	init();
	count = -1;
	resize = getParam<int>( resizeKey );
}

RangeImgReader::~RangeImgReader() {}

ColorCloud::Ptr RangeImgReader::matToCloud(const cv::Mat &colorImg, const cv::Mat &depthImg) {

	TRACE(logger, "MatToCloud: Starting");
	int height = depthImg.rows;
	int width = depthImg.cols;

	if( height != colorImg.rows || width != colorImg.cols ) {

		std::runtime_error e("Images have different dimensions");
		ERROR(logger, "MatToCloud: " << e.what() );
		throw e;
	}
	ColorCloud::Ptr cloud( new ColorCloud() );
	cloud->height = height;
	cloud->width = width;
	cloud->reserve(height * width);
	cloud->is_dense = false;

	pcl::PointXYZRGB newPoint;
	for (int y = 0; y < height; y++) {

		// For the sake of simplicity
		typedef ushort MatType;
		const MatType *depthPtr = depthImg.ptr<MatType>(y);
		const cv::Vec3b *colorPtr = colorImg.ptr<cv::Vec3b>(y);

		for (int x = 0; x < width; x++) {
			float depth = depthPtr[x] / 1000.0f;

			if ( depth == depth) {            // if depthValue is not NaN

				newPoint.z = depth;
				newPoint.x = depth * depthH * (x / depthWidthHalf);
				newPoint.y = depth * depthV * (y / depthHeightHalf);
				cv::Vec3b vec = colorPtr[x];
				newPoint.r = vec[2];
				newPoint.g = vec[1];
				newPoint.b = vec[0];

			} else {

				newPoint.z = std::numeric_limits<MatType>::quiet_NaN();
				newPoint.x = std::numeric_limits<MatType>::quiet_NaN();
				newPoint.y = std::numeric_limits<MatType>::quiet_NaN();
				newPoint.r = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.g = std::numeric_limits<unsigned char>::quiet_NaN();
				newPoint.b = std::numeric_limits<unsigned char>::quiet_NaN();
			}
			cloud->push_back(newPoint);
		}
	}
	TRACE(logger, "MatToCloud: Finished");
	std::vector<int> vec;
	pcl::removeNaNFromPointCloud( *cloud, *cloud, vec );


//	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//	viewer.showCloud (cloud);
//	while (!viewer.wasStopped ())
//	{
//	}

	return cloud;
}

void RangeImgReader::readImg(const std::string &colorPath, const std::string &depthPath, cv::Mat &colorImg, cv::Mat &depthImg) {

	TRACE(logger, "readImg: Starting");
	DEBUG(logger, "ColorImg = " << colorPath << " DepthImg = " << depthPath);
	depthImg = cv::imread(depthPath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	if( !depthImg.data ) {

		std::runtime_error e("Cloud not read image " + depthPath);
		ERROR(logger, "readImg: " << e.what() );
		throw e;
	}

	colorImg = cv::imread(colorPath);
	if( !colorImg.data ) {

		std::runtime_error e("Cloud not read image " + colorPath);
		ERROR(logger, "readImg: " << e.what() );
		throw e;
	}

	if( resize > 0) {

		int width = colorImg.cols;
		int height = colorImg.rows;
		float ratio = float(width)/height;

		if( height >= width && height > resize ) {

			height = resize;
			width = height * ratio;
		} else if ( width > resize ) {

			width = resize;
			height = width / ratio;
		}
		TRACE(logger, "width = " << width << " height = " << height);
		cv::resize(colorImg, colorImg, cv::Size(width, height), 0, 0, CV_INTER_CUBIC);
		cv::resize(depthImg, depthImg, cv::Size(width, height), 0, 0, CV_INTER_CUBIC);
	}
	TRACE(logger, "readImg: Finished");
}

ColorCloud::Ptr RangeImgReader::readImg(const std::string &colorPath, const std::string &depthPath) {

	cv::Mat colorImg, depthImg;
	readImg(colorPath, depthPath, colorImg, depthImg);
	return matToCloud(colorImg, depthImg);

}

ColorCloud::Ptr RangeImgReader::readImg() {

	DEBUG(logger, "Img #" << count);
	DEBUG(logger, "depthImgVec size = " << colorImgVec.size());
	ColorCloud::Ptr cloud;

	count++;
	if(count < colorImgVec.size())
		cloud = readImg(colorImgVec.at(count), depthImgVec.at(count));
	else
		cloud = ColorCloud::Ptr(new ColorCloud());

	return cloud;
}

int RangeImgReader::readLabel() {

	DEBUG(logger, "readLabel");
	if(count  < colorImgVec.size())
		return atoi(labelVec[count].c_str());
	return -1;
}

void RangeImgReader::setMode(int mode) {

	if(currentMode != mode) {
		currentMode = mode;
		count = -1;

		switch(currentMode) {
			case TRAIN:
				colorImgVec = getLineList( getParam<std::string>(trainColorImg) );
				depthImgVec = getLineList( getParam<std::string>(trainDepthImg) );
				labelVec = getLineList( getParam<std::string>(trainLabel));
				break;
			case TEST:
				colorImgVec = getLineList( getParam<std::string>(testColorImg) );
				depthImgVec = getLineList( getParam<std::string>(testDepthImg) );
				labelVec = getLineList( getParam<std::string>(testLabel));
				break;
			default:
				std::runtime_error e("Invalid mode");
				ERROR(logger, "ImgReader: " << e.what());
				throw e;
			}
	}

}

} /* namespace Tagger3D */
