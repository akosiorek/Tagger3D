/*
 * Cluster.h
 *
 *  Created on: 		20-06-2013
 *  	Author:			Adam Kosiorek
 *	Description:
 */

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "../Common/ProcessObject.h"

#include <opencv2/core/core.hpp>

#include <vector>

namespace Tagger3D {
/**
 *	The class is a pure virtual class - an interface for clasterization.
 */
class Cluster : public ProcessObject {
public:


	/**
	 *	Parametric constructor.
	 *	@param	_configMap	A map of configuration parameters
	 *	@return	void
	 *	@throws std::invalid_argument
	 */
	Cluster(const std::map<std::string, std::string> &_configMap);

	/**
	 * Default destructor.
	 */
	virtual ~Cluster();

	/**
	 *	The method computes a bag of words description of an input image.
	 *	@param	descriptors	A set of descriptors for which a bag of words description is to be found.
	 *	@return cv::Mat	A vector of visual words.
	 */
	virtual const std::vector<int> cluster(const cv::Mat &descriptors) = 0;

	/**
	 *	An overloaded method for multiple images.
	 *	@param	descriptors	A vector of cv::Mat, each mat containing descriptors
	 *	@return 	Each vector contains a vector of words
	 */
	virtual const std::vector<std::vector<int>> cluster(const std::vector<cv::Mat> &descriptors) = 0;


	/**
	 *	The method trains the clasterizator.
	 *	@iparam	nputData	a cv::Matrix of [number of descriptor] x [number of points].
	 *				If points from multiple images are passed than:
	 *				number of points = number of points in a single image x number of images.
	 *	@return	bool	true if the training succeded, false otherwise.
	 */
	virtual bool train(const std::vector<cv::Mat> &descriptors) = 0;

	/**
	 *	An overloaded method for a single cv::Mat.
	 *	@param descriptors	a cv::Mat of descriptors
	 *	@return	true if training was successful, false otherwise
	 */
	virtual bool train(const cv::Mat &descriptors) = 0;

	/**
	 *	Getter for labels.
	 *	@return	cv::Mat
	 */
	const cv::Mat getLabels() {return labels;};

	/**
	 *	Getter for centers.
	 *	@return cv::Mat
	 */
	const cv::Mat getCentroids() {return centroids;};

	/**
	 *	The method saves the KMeans (trained model and configuration) to a folder
	 * 	specified in a config file.
	 * 	In case the folder does not exist it will be created.
	 * 	Note that there should be only one KMeans per folder stored.
	 * 	@ return true if save successful, false otherwise
	 */
	virtual bool save() = 0;


	/**
	 *	The method loads the KMeans(trained model and configuration) from a folder
	 *	specified in a config file.
	 *	Note that there should be only one KMeans per folder.
	 *	@return	true if load successful, false otherwise
	 */
	virtual bool load() = 0;

	virtual bool isLoaded() = 0;

protected:

	cv::Mat labels;
	cv::Mat centroids;

	const std::string moduleName = "cluster" + separator;
	bool loaded;

private:
	const std::string loggerName = "Main.Cluster";

};

} /* namespace Tagger3D */
#endif /* CLUSTER_H_ */
