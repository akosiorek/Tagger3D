/*
 * PcdReader.cpp
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "PcdReader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <assert.h>

namespace Tagger3D {

PcdReader::PcdReader(const std::map<std::string, std::string> &configMap) : ImgReader(configMap) {

	init();
	voxelGrid = std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZRGB>>(new pcl::VoxelGrid<pcl::PointXYZRGB>());
	assert(voxelGrid != nullptr);
	leaf = getParam<float>( leafSize );
	voxelGrid->setLeafSize(leaf, leaf, leaf);

	count = -1;
}

PcdReader::~PcdReader() {}

ColorCloud::Ptr PcdReader::readImg(const std::string& pcdPath) {

	ColorCloud::Ptr cloud(new ColorCloud());
	pcl::io::loadPCDFile(pcdPath, *cloud);

//	for(const auto& point : cloud->points)
//		std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;

//	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//	viewer.showCloud (cloud);
//	while (!viewer.wasStopped ())
//	{
//	}

	if(leaf != 0) {

		voxelGrid->setInputCloud(cloud);
		voxelGrid->filter(*cloud);
	}

//	for(const auto& point : cloud->points)
//		std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << std::endl;




	//std::terminate();
	DEBUG(logger, "Cloud size = " << cloud->size());
	return cloud;
}

ColorCloud::Ptr PcdReader::readImg() {

	count++;
	ColorCloud::Ptr ptr;
	if(count < pcdVec.size())
		ptr = readImg(pcdVec[count]);
	else
		ptr = ColorCloud::Ptr(new ColorCloud());

	if(ptr->empty() && count < pcdVec.size()) {

		std::runtime_error e("Corrupted point cloud #" + count);
		ERROR(logger, "readImg: " << e.what());
		throw e;
	}

	return ptr;
}

int PcdReader::readLabel() {

	if(count < labelVec.size())
		return atoi(labelVec[count-1].c_str());
	return -1;
}

void PcdReader::setMode(int mode) {

	if(currentMode != mode) {
		currentMode = mode;
		count = -1;

		switch(currentMode) {
			case TRAIN:
				pcdVec = getLineList( getParam<std::string>(trainPcd));
				labelVec = getLineList( getParam<std::string>(trainPcdLabels));
				break;
			case TEST:
				pcdVec = getLineList( getParam<std::string>(testPcd) );
				labelVec = getLineList( getParam<std::string>(testPcdLabels));
				break;
			default:
				std::runtime_error e("Invalid mode");
				ERROR(logger, "ImgReader: " << e.what());
				throw e;
			}
	}

}

} /* namespace Tagger3D */
