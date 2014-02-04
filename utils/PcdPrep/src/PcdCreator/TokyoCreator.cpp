/*
 * TokyoCreator.cpp
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "TokyoCreator.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <stdexcept>

namespace fs = boost::filesystem;

TokyoCreator::TokyoCreator() {
	// TODO Auto-generated constructor stub

}

TokyoCreator::~TokyoCreator() {
	// TODO Auto-generated destructor stub
}

ColorCloud::Ptr TokyoCreator::createPointCloud(
			const std::string &jpgPath, const std::string &csvPath) {

	cv::Mat colorImg = cv::imread(jpgPath, -1);
	cv::Mat spaceImg = csv2mat(csvPath);

	if( !colorImg.data )
		throw std::runtime_error("Cloud not read image " + jpgPath);
	if( !spaceImg.data )
		throw std::runtime_error("Cloud not read image " + csvPath);

	int height = spaceImg.rows;
	int width = spaceImg.cols;
	if( spaceImg.rows != colorImg.rows || spaceImg.cols != colorImg.cols )
		throw std::runtime_error("Images have different dimensions");

	ColorCloud::Ptr cloud( new ColorCloud() );
	cloud->height = height;
	cloud->width = width;
	cloud->reserve(height * width);
	cloud->is_dense = false;
	pcl::PointXYZRGB newPoint;
	for (int i = 0; i < height; i++) {

		cv::Vec3b *spacePtr = spaceImg.ptr<cv::Vec3b>(i);
		cv::Vec3b *colorPtr = colorImg.ptr<cv::Vec3b>(i);

		for (int j = 0; j < width; j++) {

				cv::Vec3b colorVec = colorPtr[j];
				cv::Vec3b spaceVec = spacePtr[j];

				newPoint.x = spaceVec[0];
				newPoint.y = spaceVec[1];
				newPoint.z = spaceVec[2];

				newPoint.r = colorVec[2];
				newPoint.g = colorVec[1];
				newPoint.b = colorVec[0];

			cloud->push_back(newPoint);
		}
	}
	std::vector<int> vec;
	pcl::removeNaNFromPointCloud( *cloud, *cloud, vec );
	return cloud;
}

cv::Mat TokyoCreator::csv2mat(const std::string &csvPath) {

	std::ifstream file(csvPath);
	std::string line;
	std::vector<std::string> splitted;
	cv::Mat mat(width, height, CV_16UC3);
	int i = 0;
	while( !file.eof() ) {

		std::getline(file, line);
		boost::split(splitted, line, boost::is_any_of(","));
		cv::Vec3b* dPtr = mat.ptr<cv::Vec3b>(i);
		std::string *vecPtr = &splitted[0];
		for(int j = 0; j < width; j++) {
			for(int k = 0; k < 3; k++) {
				if( (*vecPtr).size() < 4)
					continue;
				dPtr[j][k] = std::atof((*vecPtr++).c_str());
			}
		}
		++i;
	}
	return mat;
}

int TokyoCreator::savePcdToFile(ColorCloud::Ptr cloud, const std::string &path) {

	pcl::io::savePCDFileBinaryCompressed(path, *cloud);
}

int TokyoCreator::batchProcess
	(const std::string &colorPath, const std::string &spacePath, const std::string &destPath) {


	fs::path colorpath(colorPath);
	fs::path spacepath(spacePath);
	fs::path destpath(destPath);
	if( !fs::exists(spacepath) ) {
		throw std::runtime_error("Path " + spacepath.string() + " does not exst");
	} else if ( !fs::is_directory(spacepath) ) {
		throw std::runtime_error("Path " + spacepath.string() + " is not a directory");
	}
	if( !fs::exists(colorpath) ) {
		throw std::runtime_error("Path " + colorpath.string() + " does not exst");
	} else if ( !fs::is_directory(colorpath) ) {
		throw std::runtime_error("Path " + colorpath.string() + " is not a directory");
	}
	if( !fs::exists(destPath) ) {
		fs::create_directory(destPath);
		std::cout << "Created " << destPath << std::endl;
	}

	fs::directory_iterator endItr;
	cv::Mat mat(width, height, CV_16UC1);
	fs::directory_iterator coloritr(colorpath);
	fs::directory_iterator spaceitr(spacepath);
	for(; coloritr != endItr && spaceitr != endItr; ++coloritr, ++spaceitr) {

		fs::path filename = coloritr->path().filename().replace_extension(extension);
		std::cout << "Processing: " << filename.string() << std::endl;
		fs::path dest = destpath / filename;
		ColorCloud::Ptr cloud = createPointCloud(coloritr->path().string(), spaceitr->path().string());
		savePcdToFile(cloud, dest.string());
	}
}
