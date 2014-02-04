/*
 * Tagger3D.cpp
 *
 *   Created on: 23 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "Tagger3D.h"
#include "Factory.h"
#include "IoUtils.h"

namespace Tagger3D {

Tagger3D::Tagger3D(const std::map<std::string, std::string> &configMap) : ProcessObject(configMap) {

	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Tagger3D");

	io = IoUtils::getInstance();
	io->setPath(directory);

	Factory f(configMap);
	imgReader = f.getReader();
	pointNormal = f.getPointNormal();
	detector = f.getDetector();
	descriptor = f.getDescriptor();
	cluster = f.getCluster();
	predictor = f.getPredictor();
}

Tagger3D::~Tagger3D() {
	DEBUG(logger, "Destroying Tagger3D");
}

int Tagger3D::predict(const std::string& rgbPath,
		const std::string& depthPath) {

	cluster->load();
	predictor->load();

	ColorCloud::Ptr colorCloud;// = imgReader->readImg(rgbPath, depthPath);
	NormalCloud::Ptr normalCloud = pointNormal->computeNormals( colorCloud );
	ScaleCloud::Ptr keypointCloud = detector->detect( colorCloud );
	cv::Mat descriptors = descriptor->describe( colorCloud, keypointCloud, normalCloud);

	cv::Mat wordDescriptors = cluster->cluster(descriptors);
	return getCatNum(predictor->predict(wordDescriptors));
}

std::vector<cv::Mat> Tagger3D::computeDescriptors() {
	INFO(logger, "Computing descriptors");

	std::vector<cv::Mat> descriptors;

	ColorCloud::Ptr colorCloud;
	NormalCloud::Ptr normalCloud;
	ScaleCloud::Ptr keypointCloud;
	cv::Mat descMat;
	int counter = 0;
	colorCloud = imgReader->readImg();

	 while(!colorCloud->empty()) {

		TRACE(logger, "points: " << colorCloud->size())

		normalCloud = pointNormal->computeNormals(colorCloud);
		pointNormal->cleanupInputCloud(colorCloud);
		TRACE(logger, "after cleanup: " << colorCloud->size());

		keypointCloud = detector->detect(colorCloud);
		TRACE(logger, "keypoints: " << keypointCloud->size());

		if(keypointCloud->size() == 0) {

			std::runtime_error e("Couldn't find any keypoints in a pointcloud #" + std::to_string(counter));
			ERROR(logger, "computeDescriptors: " << e.what());
			throw e;
		}

		descMat = descriptor->describe(colorCloud, keypointCloud, normalCloud);
		descriptors.push_back(descMat.clone());
		colorCloud = imgReader->readImg();
		counter++;
	}

	return descriptors;
}


void Tagger3D::descRun() {
	INFO(logger, "Compute Descriptors Run");

	imgReader->setMode(ImgReader::TRAIN);
	std::vector<cv::Mat> descriptors = computeDescriptors();
	io->saveVector<cv::Mat, float>(descriptors, trainDescName);

	imgReader->setMode(ImgReader::TEST);
	descriptors = computeDescriptors();
	io->saveVector<cv::Mat, float>(descriptors, testDescName);
}

void Tagger3D::clustRun() {

	std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(trainDescName);
	cluster->train(descriptors);
	cluster->save();
}

void Tagger3D::trainRun() {
	INFO(logger, "Train Predictor Run");

	bool storeHistogram = getParam<bool>(storeHistogramKey);

	std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(trainDescName);
	imgReader->setMode(ImgReader::TRAIN);
	std::vector<int> labels = imgReader->readLabels();
	cluster->load();
	cv::Mat wordDescriptors = cluster->cluster(descriptors);

	wordDescriptors.convertTo(wordDescriptors, CV_32FC1);
	if(storeHistogram) {

		io->initTextFile(trainHistogram);
		for(int i = 0; i < wordDescriptors.rows; ++i) {
			io->appendToTextFile<int>(labels[i]);
			io->appendToTextFile<float>(wordDescriptors.row(i));
			TRACE(logger, "trainHistogram #" << i);
		}
		io->finalizeTextFile();
	}

	predictor->train(wordDescriptors, labels);
	predictor->save();
}

void Tagger3D::predRun() {
	INFO(logger, "Test Run");

	bool storeHistogram = getParam<bool>(storeHistogramKey);

	std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(testDescName);
	imgReader->setMode(ImgReader::TEST);
	std::vector<int> labels = imgReader->readLabels();

	cluster->load();
	predictor->load();

	cv::Mat wordDescriptors = cluster->cluster(descriptors);

	if(storeHistogram) {

		io->initTextFile(testHistogram);
		for(int i = 0; i < wordDescriptors.rows; ++i) {
			io->appendToTextFile<int>(labels[i]);
			io->appendToTextFile<float>(wordDescriptors.row(i));
		}
		io->finalizeTextFile();
	}

	long correct = 0;
	for(int i = 0; i < wordDescriptors.rows; ++i) {

		std::vector<float> prediction = predictor->predict(wordDescriptors.row(i));
		int cat = getCatNum(prediction);
//			std::cout << cat << "\t";
//		for(auto p : prediction)
//			std::cout << p << " ";
//		std::cout << std::endl;
		if(cat == labels[i]) correct++;
	}

	std::cout << "Average: " << float(correct) / labels.size() * 100 << "%" << std::endl;
}

void Tagger3D::run() {
	INFO(logger, "Tagger3D running");

	switch(getRunMode()) {

	case Mode::DESC: descRun(); break;
	case Mode::CLUST: clustRun(); break;
	case Mode::TRAIN: trainRun(); break;
	case Mode::PRED: predRun(); break;
	}
}

int Tagger3D::getRunMode() {

	std::string m = getParam<std::string>( mode );
	auto it = std::find(std::begin(modeStrings), std::end(modeStrings), m);
	if(it == std::end(modeStrings)) {
		std::runtime_error e("Invalid mode: " + mode);
		ERROR(logger, e.what());
		throw e;
	}
	return std::distance(std::begin(modeStrings), it);
}

int Tagger3D::getCatNum(const std::vector<float> &vec) const {

	auto it = std::max_element(std::begin(vec), std::end(vec));
	return std::distance(std::begin(vec), it);
}

} /* namespace Tagger3D */
