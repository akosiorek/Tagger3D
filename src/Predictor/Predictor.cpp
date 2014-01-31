/*
 * Predictor.cpp
 *
 *  Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 * Description: Predictor base class implementation
 */

#include "Predictor.h"

#include <opencv2/core/core.hpp>

namespace Tagger3D {

Predictor::Predictor(const std::map<std::string, std::string> &_configMap,
		const std::string predictorType)

	: ProcessObject(_configMap),
	  moduleName(predictorType + separator) {


	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Predictor");

	if( _configMap.empty() ) {

		throw std::invalid_argument("Empty configuration map");
	}
	io = IoUtils::getInstance();
	class_number = getParam<int>(class_numberKey);
	normalizationPath = getParam<std::string>(normalizationPathKey);
}

Predictor::~Predictor() {

	TRACE(logger, "Destroying Predictor");
}

void Predictor::normaliseData(cv::Mat &mat) {

	assert(!v_max.empty());
	mat = (mat / repeat(v_max,  mat.rows, 1)) * 2 - 1;
}

const cv::Mat Predictor::computeMaxValues(const cv::Mat& mat) {
//	v_max = cv::Mat(1, mat.cols, CV_32SC1);
	cv::reduce(mat, v_max, REDUCE_TO_ROW, CV_REDUCE_MAX);
	TRACE(logger, "v_max size = " << v_max.size());
	return v_max;
}

cv::Mat Predictor::confusionMatrix(const std::vector<int> &labels, const std::vector<int> &predictions) const {

	int size = labels.size();
	int classes = labels[size-1] + 1;
	const int *lPtr = &labels[0];
	const int *pPtr = &predictions[0];
	cv::Mat confusionMatrix = cv::Mat::zeros(classes, classes, CV_8UC1);
	for(int i = 0; i < size; i++) {

		confusionMatrix.at<uchar>(lPtr[i], pPtr[i]) += 1;
	}

	cv::Mat classCount;
	cv::reduce(confusionMatrix, classCount, 1, CV_REDUCE_SUM, CV_32SC1);
	cv::Mat average(1, classes, CV_32FC1);
	float avg = 0;
	for(int i = 0 ; i < classes; i++) {
		average.at<float>(0, i) = float(confusionMatrix.at<uchar>(i, i)) / classCount.at<int>(0, i) * 100;
		avg += confusionMatrix.at<uchar>(i, i);
	}
	avg /= size;

	std::cout << "Entries per class: " << std::endl << classCount << std::endl;
	std::cout << "Confusion Matrix:" << std::endl << confusionMatrix << std::endl;
	std::cout << "Averages: " << std::endl << average << std::endl;
	std::cout << "Average: " << avg * 100 << std::endl;
}

void Predictor::saveVMax() {

	io->saveCv(v_max, normalizationPath);
}

void Predictor::loadVMax() {

	v_max = io->loadCv<cv::Mat>(normalizationPath);
	dims = v_max.cols;
}

} /* namespace Tagger3D */
