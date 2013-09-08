/*
 * KMeansCluster.cpp
 *
 *  Created on: 	20-06-2013
 *  Author:			Adam Kosiorek
 *	Description:
 */

#include "KMeansCluster.h"

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
//#define NDEBUG
#include <assert.h>


namespace Tagger3D {
namespace fs = boost::filesystem;

KMeansCluster::KMeansCluster(const std::map<std::string, std::string>& _configMap)
		: Cluster(_configMap){

	TRACE(logger,"configuring" );
	kMeans = nullptr;
	descriptorMatcher = nullptr;

	clusterCount = getParam<int>(clusterCountKey);
	criteriaEps = getParam<float>(criteriaEpsKey);
	criteriaItr = getParam<int>(criteriaItrKey);
	attempts = getParam<int>(attemptsKey);
	flags = getParam<int>(flagsKey);
	matcherType = getParam<std::string>(matcherTypeKey);
	centroidIoName = getParam<std::string>(centroidIoNameKey);
	ioFileFormat = getParam<std::string>(ioFileFormatKey);

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
}

KMeansCluster::~KMeansCluster() {}

bool KMeansCluster::createKMeans() {

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
	return true;
}

bool KMeansCluster::createDescriptorMatcher() {

	TRACE(logger,"createDescriptorMatcher: Starting" );
	descriptorMatcher = cv::DescriptorMatcher::create(matcherType);
	TRACE(logger,"createDescriptorMatcher: Finished" );
	return true;
}

const std::vector<int> KMeansCluster::cluster(const cv::Mat &descriptors) {

	TRACE(logger,"cluster: Starting" );

	if( loaded == false) {

		std::logic_error e("KMeans has not been trained");
		ERROR(logger, "cluster: " << e.what());
		throw e;
	}

	if( descriptorMatcher == nullptr ) {

		createDescriptorMatcher();
		assert(descriptorMatcher != nullptr);
	}

	std::vector<cv::DMatch> matches;

	TRACE(logger,"cluster: Matching descriptors" );
//	TRACE(logger, "cluster: descriptors = " << descriptors)
	descriptorMatcher->match(descriptors, centroids, matches);

	std::vector<int> visualWords;
	visualWords.resize(matches.size());

	TRACE(logger,"cluster: parsing results" );
	for(int i = 0; i < matches.size(); i++)
		visualWords[ matches[i].queryIdx ] = matches[i].trainIdx + 1;

//	for(auto &word : visualWords)
//		std::cout << word << " ";

	TRACE(logger,"cluster: Finished - visualWords size = " << visualWords.size() );
	return visualWords;
}

const std::vector<std::vector<int>> KMeansCluster::cluster(const std::vector<cv::Mat> &descriptors) {

	std::vector<std::vector<int>> matched;
	matched.reserve(descriptors.size());

	TRACE(logger, "descriptors size = " << descriptors.size());
	for(int i = 0; i < descriptors.size(); i++)
		matched.push_back(cluster(descriptors[i]));

	return matched;
}

bool KMeansCluster::train(const std::vector<cv::Mat> &descriptors) {

	TRACE(logger,"train: Starting" );
	if( kMeans == nullptr) {

		createKMeans();
		assert( kMeans != nullptr );
	}

	TRACE(logger,"train: Adding descriptors" );
	for(int i = 0; i < descriptors.size(); i++) {

		kMeans->add(descriptors[i]);
	}

	TRACE(logger,"train: Clustering" );
	centroids = kMeans->cluster();
	loaded = true;

	TRACE(logger,"train: Finished - centroids size = " << centroids.size() );
	return true;
}

bool KMeansCluster::train(const cv::Mat &descriptors) {

	std::vector<cv::Mat> matVector;
	matVector.push_back(descriptors);
	train(matVector);
}

bool KMeansCluster::save() {

	TRACE(logger,"save: Starting" );
	if( kMeans == nullptr) {

		std::logic_error e("KMeans has not been initialised");
		ERROR(logger, "save: " << e.what());
		throw e;
	}

	if( loaded == false) {

		std::logic_error e("KMeans has not been trained");
		ERROR(logger, "save: " << e.what());
		throw e;
	}

	fs::path path(directory);
	if( !fs::exists( path ) ) {
		fs::create_directories(path);
		if(  !fs::exists( path )) {

			std::runtime_error e("Could not create the folder at: " + path.string());
			ERROR(logger, "save: " << e.what() );
			throw e;
		}
	}
	std::string name = directory + "/" + centroidIoName + ioFileFormat;  	// path to XML file
	cv::FileStorage fs(name, cv::FileStorage::WRITE); 					// open file in write format
	if( !fs.isOpened()) {

		std::runtime_error e("Cannot initialise a filestorage at " + name);
		ERROR(logger, "save: " << e.what() );
		throw e;
	}
	/*
	 * In order to write to fs one has to specify key-value pairs in the following way:
	 * fs << key << value;
	 * The value can be of any type. Unfortunately, I could not figure out what type the key should be. Providing :
	 * fs << "string" << value works. But as soon as I try replace "string" with a variable (or a const) of a type std::string or char
	 * it stops working.
	 * For that reason serialization of training keypoints has been halted. Serialization of centriods works perfectly fine.
	 *
	for(int i = 0; i < descriptors.size(); i++) {

		//cv::Mat mat = descriptors[i];
		//fs << std::to_string(i) << mat;
		const char* key = std::to_string(i).c_str();
		fs << key << 5;
	}
	*/

	TRACE(logger,"save: Saving" );
	fs << centroidIoName << centroids;  // save centroid.xml
	TRACE(logger,"save: Releasing filestorage" );
	fs.release();

	TRACE(logger,"save: Finished" );
	return true;
}

bool KMeansCluster::load() {

	TRACE(logger,"load: Starting" );
	fs::path path(directory);
	if( !fs::exists(path) ) {

		std::runtime_error e ("The specified path " + path.string() + " does not exist.");
		ERROR(logger, "load: " << e.what() );
		throw e;
	}

	if( !fs::is_directory(path) ) {

		std::runtime_error e ("The specified path " + path.string() + " is not a directory");
		ERROR(logger, "load: " << e.what() );
		throw e;
	}

	if( kMeans == nullptr ) {

		createKMeans();
		assert( kMeans != nullptr );
	}

	TRACE(logger,"load: Opening file storage" );
	std::string name = directory + "/" + centroidIoName + ioFileFormat;
	cv::FileStorage fs(name, cv::FileStorage::READ);
	if( !fs.isOpened()) {
		fs.open(name, cv::FileStorage::READ);
	}
	if (!fs.isOpened()) {

		std::runtime_error e("Could not open the following file: " + name);
		ERROR(logger, "load: " << e.what() );
		throw e;
	}

	TRACE(logger,"load: Loading centroids" );
	fs[centroidIoName] >> centroids; // load from xml file
	loaded = true;

	TRACE(logger,"load: Finished" );
	return true;
}

} /* namespace Tagger3D */

