/*
 * KMeansCluster.h
 *
 *  Created on: 	20-06-2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#ifndef KMEANSCLUSTER_H_
#define KMEANSCLUSTER_H_

#include "Cluster.h"

#include <opencv2/features2d/features2d.hpp>
#include <memory>

namespace Tagger3D {

/**
 * Class implements Cluster interface. It clasterizes data with KMeans algorithm.
 * Based on OpenCV.
 */
class KMeansCluster: public Cluster {
public:

	/**
	 * Parametric constructor.
	 * @controller		controller
	 * @configFilePath	filepath to a configuration file
	 */
	KMeansCluster(const std::map<std::string, std::string>& _configMap);

	/**
	 * Default destructor.
	 */
	~KMeansCluster();

	/**
	 * The method performs clusterization.
	 * @throws std::logic_error
	 * @override
	 */
	const std::vector<int> cluster(const cv::Mat &descriptors);

	/**
	 * The method performs clusterization of muliple images.
	 * @throws std::logic_error
	 * @override.
	 */
	const std::vector<std::vector<int>> cluster(const std::vector<cv::Mat> &descriptors);

	/**
	 *	The method trains the clasterizator.
	 *	@throws std::logic_error
	 *	@override
	 */
	bool train(const std::vector<cv::Mat> &descriptors);

	/**
	 * The method trains the clasterizator;
	 * @return bool
	 * @override
	 */
	bool train(const cv::Mat &descriptors);

	/**
	 *	The method saves the KMeans.
	 *	@throws std::logic_error, std::runtime_error
	 *	@override
	 */
	bool save();

	/**
	 *	The method loads the KMeans.
	 *	@throws std::runtime_error
	 *	@override
	 */
	bool load();

	bool isLoaded(){ return loaded; }

private:

	std::unique_ptr<cv::BOWKMeansTrainer> kMeans;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

	/**
	 * KMeans parameters.
	 */
	int clusterCount;
	int criteriaEps;
	int criteriaItr;
	int attempts;
	int flags;

	/**
	 *  Allowed matcher types:
	 *  BruteForce			Uses L2 metrics
	 *  BruteForce-L1
	 *  BruteForce-Hamming
	 *  BruteForce-Hamming(2)
	 */
	std::string matcherType;
	std::string centroidIoName;
	std::string ioFileFormat;
	std::string inputOutputPath;

	/**
	 * KMeans key values. Configuration parameters' values should be stored
	 * in a configuration file as values assigned to the keys below.
	 * The supported format is one key:value pair per line.
	 */
	const std::string clusterCountKey = moduleName + "clusterCount";
	const std::string criteriaEpsKey = moduleName + "criteriaEps";
	const std::string criteriaItrKey = moduleName + "criteriaItr";
	const std::string attemptsKey = moduleName + "attempts";
	const std::string flagsKey = moduleName + "flags";
	const std::string matcherTypeKey = moduleName + "matcherType";
	const std::string centroidIoNameKey = moduleName + "centroidIoName";
	const std::string ioFileFormatKey = moduleName + "ioFileFormat";
	const std::string inputOutputPathKey = moduleName + "inputOutputPath";

	bool createKMeans();
	bool createDescriptorMatcher();
};

} /* namespace Tagger3D */
#endif /* KMEANSCLUSTER_H_ */
