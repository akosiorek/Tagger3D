/*
 * PcdReader.cpp
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "PcdReader.h"


#include <pcl/io/pcd_io.h>

namespace Tagger3D {

PcdReader::PcdReader(const std::map<std::string, std::string> &configMap) : ImgReader(configMap) {

	count = 0;
	leaf = getParam<float>( leafSize );
	chunkSize = getParam<int>( chunkSizeKey );
	voxelGrid = std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZRGB>>(new pcl::VoxelGrid<pcl::PointXYZRGB>());
	voxelGrid->setLeafSize(leaf, leaf, leaf);

	switch( getParam<int>( mode )) {
	case TRAIN:
		pcdVec = getLineList( getParam<std::string>( trainPcd ));
		labelVec = getLineList( getParam<std::string>( trainPcdLabels ));
		break;
	case TEST:
		pcdVec = getLineList( getParam<std::string>( testPcd ));
		labelVec = getLineList( getParam<std::string>( trainPcdLabels ));
		break;
	default:
		std::runtime_error e("Invalid mode");
		ERROR(logger, "ImgReader: " << e.what());
		throw e;
	}
}

PcdReader::~PcdReader() {}

ColorCloud::Ptr PcdReader::readImg(const std::string& pcdPath) {

	ColorCloud::Ptr cloud(new ColorCloud());
	pcl::io::loadPCDFile(pcdPath, *cloud);

	if(leaf != 0) {

		voxelGrid->setInputCloud(cloud);
		voxelGrid->filter(*cloud);
	}
	DEBUG(logger, "Cloud size = " << cloud->size());
	return cloud;
}

ColorVec PcdReader::readImgs() {

	TRACE(logger, "readImgs: Starting");

	ColorVec clouds;
	int limit = count + chunkSize;
	if( limit > pcdVec.size()) {

		limit = pcdVec.size();
	}

	while( count < limit) {
		clouds.push_back( readImg() );
	}
	TRACE(logger, "readImgs: Finished");
	return clouds;
}

ColorCloud::Ptr PcdReader::readImg() {

	return readImg( pcdVec[count++] );
}

int PcdReader::readLabel() {

	return 0;
}

} /* namespace Tagger3D */
