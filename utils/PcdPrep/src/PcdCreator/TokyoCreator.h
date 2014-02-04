/*
 * TokyoCreator.h
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef TOKYOCREATOR_H_
#define TOKYOCREATOR_H_

#include <opencv2/core/core.hpp>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;

/*
 *
 */
class TokyoCreator {
	/**
	 *
	 */
public:
	TokyoCreator();
	virtual ~TokyoCreator();

	ColorCloud::Ptr createPointCloud(
			const std::string &jpgPath, const std::string &csvPath);

	int savePcdToFile(ColorCloud::Ptr cloud, const std::string &path);

	int batchProcess(const std::string &colorPath, const std::string &spacePath, const std::string &destPath);

private:

	cv::Mat csv2mat(const std::string &csvPath);

	const int width = 480;
	const int height = 640;
	const std::string extension = ".pcd";

};
#endif /* TOKYOCREATOR_H_ */
