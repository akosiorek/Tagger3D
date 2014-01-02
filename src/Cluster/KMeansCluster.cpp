/*
 * KMeansCluster.cpp
 *
 *  Created on: 	20-06-2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#include "KMeansCluster.h"
#include "IoUtils.h"

#include <opencv2/opencv.hpp>
//#define NDEBUG
#include <assert.h>

namespace Tagger3D {

KMeansCluster::KMeansCluster(const std::map<std::string, std::string>& _configMap)
		: Cluster(_configMap){

	TRACE(logger,"configuring" );
	kMeans = nullptr;
	descriptorMatcher = nullptr;

	criteriaEps = getParam<float>(criteriaEpsKey);
	criteriaItr = getParam<int>(criteriaItrKey);
	attempts = getParam<int>(attemptsKey);
	flags = getParam<int>(flagsKey);
	matcherType = getParam<std::string>(matcherTypeKey);
	centroidIoName = getParam<std::string>(centroidIoNameKey);

	if( attempts == 0) {
		std::invalid_argument e("Attemps should be > 0");
		ERROR(logger, "Constructor: " << e.what() );
		throw e;
	}


	if( criteriaEps == 0 && criteriaItr == 0) {

		std::invalid_argument e("Invalid end criteria");
		ERROR(logger, "Constructor: " << e.what() );
		throw e;
	}

	createKMeans();
	createDescriptorMatcher();
}

void KMeansCluster::createKMeans() {

	TRACE(logger,"createKMeans: Starting" );
	cv::TermCriteria termCriteria;
	if(criteriaEps == 0) {

		termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT, criteriaItr, criteriaEps);
	} else if(criteriaItr == 0) {

		termCriteria = cv::TermCriteria(cv::TermCriteria::EPS, criteriaItr, criteriaEps);
	} else {

		termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, criteriaItr, criteriaEps);
	}

	kMeans = std::unique_ptr<cv::BOWKMeansTrainer>(new cv::BOWKMeansTrainer(clusterCount, termCriteria, attempts, flags));
	TRACE(logger,"createKMeans: Finished" );
}

void KMeansCluster::createDescriptorMatcher() {

	TRACE(logger,"createDescriptorMatcher: Starting" );
	descriptorMatcher = cv::DescriptorMatcher::create(matcherType);
	TRACE(logger,"createDescriptorMatcher: Finished" );
}

cv::Mat KMeansCluster::cluster(const cv::Mat &descriptors) {

	TRACE(logger,"cluster: Starting" );

	if( loaded == false) {

		std::logic_error e("KMeans has not been trained");
		ERROR(logger, "cluster: " << e.what());
		throw e;
	}

	std::vector<cv::DMatch> matches;
	descriptorMatcher->match(descriptors, centroids, matches);
	TRACE(logger,"cluster: Finished");
	return makeHistogram(matches);

//	long matchesSize = matches.size();
//	std::vector<int> visualWords(matchesSize);
//	int* vPtr = &visualWords[0];
//	auto *mPtr = &matches[0];
//
//	for(int i = 0; i < matchesSize; i++)
//		vPtr[ matches[i].queryIdx ] = mPtr[i].trainIdx + 1;
//
//	TRACE(logger,"cluster: Finished - visualWords size = " << visualWords.size() );
//	return makeHistogram(visualWords);
}

void KMeansCluster::train(const std::vector<cv::Mat> &descriptors) {

	TRACE(logger,"train: Starting" );

	for(int i = 0; i < descriptors.size(); i++)
		kMeans->add(descriptors[i]);

	TRACE(logger,"train: Clustering" );
	centroids = kMeans->cluster();
	loaded = true;

	TRACE(logger,"train: Finished - centroids size = " << centroids.size() );
}

cv::Mat KMeansCluster::makeHistogram(const std::vector<int> &vec) const {

	cv::Mat hist = cv::Mat::zeros(1, clusterCount, CV_32FC1);
	auto *hPtr = hist.ptr<float>();
	const int *vPtr = &vec[0];
	int size = vec.size();
	for(int i = 0; i < size ;++i)
		hPtr[vPtr[i]] = hPtr[vPtr[i]] + 1;

	return hist;
}

cv::Mat KMeansCluster::makeHistogram(const std::vector<cv::DMatch> &matches) const {

	cv::Mat hist = cv::Mat::zeros(1, clusterCount, CV_32FC1);
	auto *hPtr = hist.ptr<float>();
	const cv::DMatch *mPtr = &matches[0];

	long matchesSize = matches.size();

	for(int i = 0; i < matchesSize; i++)
			hPtr[mPtr[i].trainIdx] +=  1;

	return hist;
}

void KMeansCluster::save() {

	TRACE(logger, "save: Saving kMeans");
	IoUtils::getInstance()->saveMatBinary<float>(centroids, centroidIoName);
}

void KMeansCluster::load() {

	TRACE(logger, "load: Loading kMeans");
	centroids = IoUtils::getInstance()->loadMatBinary<float>(centroidIoName, clusterCount, dimCount);
	loaded = true;
}

} /* namespace Tagger3D */

