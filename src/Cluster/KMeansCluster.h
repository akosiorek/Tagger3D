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

	KMeansCluster() = delete;

	/**
	 * Parametric constructor.
	 * @controller		controller
	 * @configFilePath	filepath to a configuration file
	 */
	KMeansCluster(const std::map<std::string, std::string>& _configMap);

	/**
	 * Default destructor.
	 */
	~KMeansCluster() = default;


	cv::Mat cluster(const cv::Mat &descriptors) override;
	void train(const std::vector<cv::Mat> &descriptors) override;
	void save() override;
	void load() override;
	bool isLoaded() override { return loaded; };

private:

	std::unique_ptr<cv::BOWKMeansTrainer> kMeans;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	cv::Mat centroids;
	cv::Mat makeHistogram(const std::vector<int> &vec) const;
	cv::Mat makeHistogram(const std::vector<cv::DMatch> &vec) const;

	/**
	 * KMeans parameters.
	 */

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

	/**
	 * KMeans key values. Configuration parameters' values should be stored
	 * in a configuration file as values assigned to the keys below.
	 * The supported format is one key:value pair per line.
	 */
	const std::string criteriaEpsKey = moduleName + "criteriaEps";
	const std::string criteriaItrKey = moduleName + "criteriaItr";
	const std::string attemptsKey = moduleName + "attempts";
	const std::string flagsKey = moduleName + "flags";
	const std::string matcherTypeKey = moduleName + "matcherType";
	const std::string centroidIoNameKey = moduleName + "centroidIoName";

	void createKMeans();
	void createDescriptorMatcher();
};

} /* namespace Tagger3D */
#endif /* KMEANSCLUSTER_H_ */
