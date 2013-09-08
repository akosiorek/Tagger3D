/*
 * Tagger3D.cpp
 *
 *   Created on: 23 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "Tagger3D.h"
#include "../ImgReader/RangeImgReader.h"
#include "../ImgReader/PcdReader.h"
#include "../PointNormal/NormalEstimator.h"
#include "../Detector/SIFTDetector.h"
#include "../Descriptor/PFHDescriptor.h"
#include "../Descriptor/FPFHDescriptor.h"
#include "../Cluster/KMeansCluster.h"
#include "../Predictor/SldaPredictor.h"
#include "../Predictor/SVMPredictor.h"

#include <assert.h>

namespace Tagger3D {

Tagger3D::Tagger3D(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap) {

	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Tagger3D");

	switch( getParam<int>( readerType )) {
	case RANGEIMG_READER: imgReader = std::unique_ptr<ImgReader>(new RangeImgReader(configMap)); break;
	case PCD_READER: imgReader = std::unique_ptr<ImgReader>(new PcdReader(configMap)); break;
	default:
		std::runtime_error e("Invalid reader type");
		ERROR(logger, e.what());
		throw e;
	}

	pointNormal = std::unique_ptr<PointNormal> (new NormalEstimator(configMap));
	detector = std::unique_ptr<Detector> (new SIFTDetector(configMap));

	switch( getParam<int>( descType )) {
	case PFH_DESC: descriptor = std::unique_ptr<Descriptor> (new PFHDescriptor(configMap)); break;
	case FPFH_DESC: descriptor = std::unique_ptr<Descriptor> (new FPFHDescriptor(configMap)); break;
	default:
		std::runtime_error e("Invalid descriptor type");
		ERROR(logger, e.what());
		throw e;
	}

	cluster = std::unique_ptr<Cluster> (new KMeansCluster(configMap));

	switch( getParam<int>( predictorType )) {
	case SLDA: predictor = std::unique_ptr<Predictor> (new SldaPredictor(configMap)); break;
	case SVM: predictor = std::unique_ptr<Predictor> (new SVMPredictor(configMap)); break;
	default:
			std::runtime_error e("Invalid predictor type");
			ERROR(logger, e.what());
			throw e;
	}

	assert( imgReader != nullptr );
	assert( pointNormal != nullptr );
	assert( detector != nullptr );
	assert( descriptor != nullptr );
	assert( cluster != nullptr );
	assert( predictor != nullptr );
}

Tagger3D::~Tagger3D() {
	DEBUG(logger, "Destroying Tagger3D");
}

int Tagger3D::trainTest() {

	train();
	test();
	return 1;
}
void Tagger3D::train() {

	INFO(logger, "Train");
	std::vector<int> labels;
	ColorVec colorClouds;
	NormalVec normalClouds;
	ScaleVec keypointClouds;
	std::vector<cv::Mat> descriptors;
	std::vector<std::vector<int>> wordDescriptors;

	INFO(logger, "Reading images");

	colorClouds = imgReader->readImgs();
	labels = imgReader->readLabels();

	if(colorClouds.size() != labels.size()) {

		std::runtime_error e("Number of point clouds and labels differ: " +
				std::to_string(colorClouds.size()) + ", " + std::to_string(labels.size()));
		ERROR(logger, "train: " << e.what());
		throw e;
	}

	INFO(logger, "Read " << colorClouds.size() << " point clouds");
	INFO(logger, "Computing normals");

	for(auto &cloud : colorClouds) {

		normalClouds.emplace_back(pointNormal->computeNormals( cloud ));
		pointNormal->cleanupInputCloud( cloud );
	}

	INFO(logger, "Detecting keypoints");
	keypointClouds = detector->detect( colorClouds );

	INFO(logger, "Describing keypoints");
	descriptors = descriptor->describe( colorClouds, keypointClouds, normalClouds );




	switch( getParam<int>(trainCluster) ) {
	case 1:
		INFO(logger, "Clusterizing");
		cluster->train(descriptors);
		cluster->save();
		break;
	case 0:
		INFO(logger, "Loading centroids");
		cluster->load();
		break;
	default:
		std::runtime_error e("Invalid cluster operation mode");
		ERROR(logger, e.what());
		throw e;
	}



	INFO(logger, "Generating word descriptions");
	wordDescriptors = cluster->cluster(descriptors);

	INFO(logger, "Training classificator");
	predictor->train(wordDescriptors, labels);
}

void Tagger3D::test() {

	INFO(logger, "Test");
	std::vector<int> labels;
	ColorVec colorClouds;
	NormalVec normalClouds;
	ScaleVec keypointClouds;
	std::vector<cv::Mat> descriptors;
	std::vector<std::vector<int>> wordDescriptors;

	INFO(logger, "Reading images");
	colorClouds = imgReader->readImgs();
	labels = imgReader->readLabels();

	if(colorClouds.size() != labels.size()) {

			std::runtime_error e("Number of point clouds and labels differ: " +
						std::to_string(colorClouds.size()) + ", " + std::to_string(labels.size()));
			ERROR(logger, "train: " << e.what());
			throw e;
		}

	INFO(logger, "Read " << colorClouds.size() << " point clouds");
	INFO(logger, "Computing normals");
	for(auto &cloud : colorClouds) {

		normalClouds.emplace_back(pointNormal->computeNormals( cloud ));
		pointNormal->cleanupInputCloud( cloud );
	}

	INFO(logger, "Detecting keypoints");
	keypointClouds = detector->detect( colorClouds );

	INFO(logger, "Describing keypoints");
	descriptors = descriptor->describe( colorClouds, keypointClouds, normalClouds );

	if( !cluster->isLoaded() ) {

		cluster->load();
	}

	INFO(logger, "Generating word description");
	wordDescriptors = cluster->cluster(descriptors);

	INFO(logger, "Classifying");
	predictor->predict(wordDescriptors, labels);
}

int Tagger3D::predict(const std::string& rgbPath,
		const std::string& depthPath) {

	ColorCloud::Ptr colorCloud;// = imgReader->readImg(rgbPath, depthPath);
	NormalCloud::Ptr normalCloud = pointNormal->computeNormals( colorCloud );
	ScaleCloud::Ptr keypointCloud = detector->detect( colorCloud );
	cv::Mat descriptors = descriptor->describe( colorCloud, keypointCloud, normalCloud);
	cluster->load();
	std::vector<int> wordDescriptors = cluster->cluster(descriptors);
	return predictor->predict(wordDescriptors)[0];

}

int Tagger3D::run() {

	int m = getParam<int>( mode );
	switch(m) {

	case TRAIN: train(); break;
	case TEST: test(); break;
	case TRAINTEST: trainTest(); break;
	case PREDICT: ; break;
	default:
		std::runtime_error e("Unrecognized mode = " + mode);
		ERROR(logger, "run: " << e.what());
		throw e;
	}
}



} /* namespace Tagger3D */
