/*
 * PcdReader.h
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef PCDREADER_H_
#define PCDREADER_H_

#include "ImgReader.h"
#include <pcl/filters/voxel_grid.h>
namespace Tagger3D {

/*
 *
 */
class PcdReader: public ImgReader {
	/**
	 *
	 */
public:
	PcdReader(const std::map<std::string, std::string> &configMap);
	virtual ~PcdReader();

	ColorCloud::Ptr readImg();
	ColorCloud::Ptr readImg(const std::string &pcdPath);
	ColorVec readImgs();

private:
	PcdReader();
	std::unique_ptr<pcl::VoxelGrid<pcl::PointXYZRGB>> voxelGrid;
	float leaf;
	int count;
	int chunkSize;

	const std::string chunkSizeKey = moduleName + "chunkSize";
	const std::string leafSize = moduleName + "leafSize";
	const std::string trainPcd = moduleName + "trainPcdList";
	const std::string testPcd = moduleName + "testPcdList";
	const std::string trainPcdLabels = moduleName + "trainPcdLabels";
	const std::string testPcdLabels = moduleName + "testPcdLabels";

	std::vector<std::string> pcdVec;

};

} /* namespace Tagger3D */
#endif /* PCDREADER_H_ */
