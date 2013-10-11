/*
 * Tagger3D : ImgReader.cpp
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#include "ImgReader.h"
#include <opencv2/core/types_c.h>
#include <fstream>
namespace Tagger3D {

ImgReader::ImgReader(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap) {

	logger = lgr::Logger::getLogger( loggerName );
	DEBUG(logger, "Creating ImgReader");
}

ImgReader::~ImgReader() {

	DEBUG(logger, "Destroying ImgReader");
}

void ImgReader::init() {

	currentMode = -1;
	std::string stringMode = getParam<std::string>(mode);
	int tmpMode;
	bool att = false;
	if(stringMode == "train") {tmpMode = TRAIN; att = true;}
	else if(stringMode == "test") {tmpMode = TEST; att = true;}
	if(att)
		setMode(tmpMode);
	else
		setMode(getParam<int>( mode ));
}

std::vector<std::string> ImgReader::getLineList(const std::string &path) {

	TRACE(logger, "getImgList: Starting");
	if( !fileExists(path) ) {

		std::runtime_error e("Cannot open the following file " + path);
		ERROR(logger, e.what());
		throw e;
	}
	std::ifstream listFile( path );
	std::string line;
	std::vector<std::string> imgList;
	while( !listFile.eof() ) {

		std::getline(listFile, line);
		if( !line.empty() ) {
			imgList.push_back( line );
		}

	}
	TRACE(logger, "getImgList: Finished");
	return imgList;
}

std::string ImgReader::typeToStr(const int &type) {

  std:: string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

std::vector<int> ImgReader::readLabels() {
	INFO(logger, "Reading labels");

	std::vector<int> labels;
	for(const auto &str : labelVec) {

		labels.push_back(atoi(str.c_str()));
	}
	DEBUG(logger, "Read " << labels.size() << " labels");
	return labels;
}



} /* namespace Tagger3D */
