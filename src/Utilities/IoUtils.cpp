/*
 * IoUtils.cpp
 *
 *  Created on: 4 Oct 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#include "IoUtils.h"
namespace Tagger3D {

IoUtils::IoUtils()
	: logger(lgr::Logger::getLogger(loggerName)) {
	DEBUG(logger, "Creating IoUtils");
}

IoUtils::~IoUtils() {
	DEBUG(logger, "Destroying IoUtils");
}

std::shared_ptr<IoUtils> IoUtils::getInstance() {

	static std::shared_ptr<IoUtils> io = std::shared_ptr<IoUtils>(new IoUtils());
	return io;
}

std::string IoUtils::makePath(const std::string &path, const std::string &filename) const {

	return path + "/" + filename;
}

void IoUtils::saveVectorMatBinaryStats(const std::vector<cv::Mat> &vec, const std::string &filename) const {
	DEBUG(logger, "Saving std::vector<cv::Mat> statistics");


	std::string filepath = makePath(path, filename + stat);
	std::ofstream os(filepath, std::ios::binary);
	if(!os.is_open() || !os.good()) {
		std::runtime_error e("Unable to open file: " + filepath);
		ERROR(logger, "saveVectorMatBinaryStats: " << e.what());
		throw e;
	}

	int mats = vec.size();
	int cols = vec[0].cols;

	std::vector<int> sizes;
	sizes.reserve(mats);
	for(const auto &mat : vec)
		sizes.push_back(mat.rows);

	// write number of images, dims and number of keypoints per image
	os.write((char*)&mats, sizeof(int));
	os.write((char*)&cols, sizeof(int));
	os.write((char*)&sizes[0], sizes.size() * sizeof(int));
	os.close();
}

void IoUtils::initTextFile(const std::string& filename) {

	if(outStream.is_open()) {
		std::runtime_error e("Previus file has not been finalized");
		ERROR(logger, "initTextFile: " << e.what());
		throw e;
	}
	outStream.open(makePath(path, filename).c_str());
}

void IoUtils::finalizeTextFile() {

	outStream.close();
}

std::vector<int> IoUtils::loadVectorMatBinaryStats(const std::string &filename) const {
	DEBUG(logger, "Loading std::vector<cv::Mat> statistics");

	std::string filepath = makePath(path, filename + stat);
	std::ifstream is(filepath, std::ios::binary);
	if(!is.is_open() || !is.good()) {
		std::runtime_error e("Unable to open file: " + filepath);
		ERROR(logger, "loadVectorMatBinaryStats: " << e.what());
		throw e;
	}

	int mats = 0;
	int cols = 0;

	is.read((char*)&mats, sizeof(int));
	is.read((char*)&cols, sizeof(int));
	std::vector<int> sizes;
	sizes.resize(mats + 2);
	sizes[0] = mats;
	sizes[1] = cols;

	is.read((char*)&sizes[2], mats* sizeof(int));
	is.close();

	return sizes;
}

} /* namespace Tagger3D */
