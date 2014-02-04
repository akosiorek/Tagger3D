#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"
#include <stdexcept>

struct Annotation;
std::vector<std::string> getImgList(const std::string &path);
std::vector<Annotation> getAnnotations(const std::string &annotPath);
void extractObjects(const cv::Mat &img,
		const Annotation &annot, const std::string &name, const std::string &dest);

int main(int argc, char** argv) {

	if(argc < 5) {

		std::cout << "./ImgPrep <rgbImgList> <depthImgList> <annotationsList> <rgbDestFolder> <depthDestFolder>" << std::endl;
		return 2;
	}

	std::string rgbList = argv[1];
	std::string depthList = argv[2];
	std::string annotList = argv[3];
	std::string rgbDest = argv[4];
	std::string depthDest = argv[5];
	std::vector<std::string> rgbVector = getImgList(rgbList);
	std::vector<std::string> depthVector = getImgList(depthList);
	std::vector<std::string> annotVector = getImgList(annotList);

	size_t size = rgbVector.size();
	if( size != depthVector.size() ) {

		std::cout << "Different numbers of rgb and depth images" << std::endl;
		return 2;
	}

	cv::Mat rgbImg, depthImg;
	std::vector<Annotation> annotations;
	std::string* rgbData = &rgbVector[0];
	std::string* depthData = &depthVector[0];
	std::string* annotData = &annotVector[0];

	int i = 0;
	const std::string rgb = "rgb";
	const std::string depth = "depth";

	for(int i = 0; i < size ; ++i) {

		rgbImg = cv::imread(rgbData[i], -1);
		depthImg = cv::imread(depthData[i], -1);
		if( !rgbImg.data || !depthImg.data ) {

			std::cout << "Cannot read img #" << i << std::endl;
			return 2;
		}

		annotations = getAnnotations(annotData[i]);
		for(const auto &annotation : annotations) {

			extractObjects(rgbImg, annotation, rgb + std::to_string(i), rgbDest);
			extractObjects(depthImg, annotation, depth + std::to_string(i), depthDest);
		}
	}
}

struct Annotation {

	int xmax, xmin, ymax, ymin;
	std::string label;
};

std::vector<std::string> getImgList(const std::string &path) {

	std::ifstream listFile( path );
	std::string line;
	std::vector<std::string> imgList;
	while( !listFile.eof() ) {

		std::getline(listFile, line);
		if( line.empty() ) {

			continue;
		}
		imgList.emplace_back( line );
	}
	return imgList;
}

std::vector<Annotation> getAnnotations(const std::string &annotPath) {

	TiXmlDocument doc(annotPath.c_str());
	if( !doc.LoadFile() ) {

		throw std::runtime_error("Could not load file " + annotPath);
	}
	std::vector<Annotation> annots;
	Annotation singleAnnot;

	TiXmlHandle docHandle(&doc);
	TiXmlElement* element;
	TiXmlHandle rootHandle(0);

	element = docHandle.FirstChildElement().Element();
	if( !element) {

		throw std::runtime_error("No Root element in file " + annotPath);
	}
	rootHandle = TiXmlHandle(element);

	TiXmlElement *object = rootHandle.FirstChild("object").Element();
	TiXmlElement *subObject;

	for(; object; object = object->NextSiblingElement("object") ) {

		subObject = object->FirstChildElement();
		singleAnnot.label = subObject->GetText();
		subObject = subObject->NextSiblingElement()->FirstChildElement();
		singleAnnot.xmax = atoi(subObject->GetText());
		subObject = subObject->NextSiblingElement();
		singleAnnot.xmin = atoi(subObject->GetText());
		subObject = subObject->NextSiblingElement();
		singleAnnot.ymax = atoi(subObject->GetText());
		subObject = subObject->NextSiblingElement();
		singleAnnot.ymin = atoi(subObject->GetText());
		annots.push_back(singleAnnot);
	}
	return annots;
}

void extractObjects(const cv::Mat &img,
		const Annotation &annot, const std::string &name, const std::string &dest) {

	cv::Rect rect(annot.xmin, annot.ymin, annot.xmax - annot.xmin, annot.ymax - annot.ymin);
	cv::Mat roi(img, rect);
	std::string path = dest + "/" + annot.label + "_" + name + ".png";
	if( !cv::imwrite(path, roi) ) {

		throw std::runtime_error("Could not write image at " + path);
	}
}

