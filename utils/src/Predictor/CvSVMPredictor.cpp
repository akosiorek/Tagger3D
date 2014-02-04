/*
 * CvSVMPredictor.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Adam Kosiorek
 * Description: svm predictor implementation
 */

#include "CvSVMPredictor.h"

#include <stdio.h>
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iterator>

namespace Tagger3D {

CvSVMPredictor::CvSVMPredictor(const std::map<std::string, std::string> &_configMap,
		const std::string &predictorType)

	: Predictor(_configMap, predictorType) {

    svmPath = directory + "/" + getParam<std::string>( svmPathKey );
    createSVM();
}

CvSVMPredictor::~CvSVMPredictor() {

}

void CvSVMPredictor::createSVM() {

	params.svm_type    = getParam<int>( svmType );
	params.kernel_type = getParam<int>( kernelType );
	params.term_crit   = cvTermCriteria(getParam<int>(termCrit), getParam<int>(maxIter), getParam<double>(epsilon));

	params.gamma = getParam<double>( gamma );
	params.C = getParam<double>( C );
	params.degree = getParam<int>( degree );

	std::cout <<params.C << " " << params.gamma<< std::endl;
}

void CvSVMPredictor::train(cv::Mat& data, const std::vector<int>& labels) {


	TRACE(logger, "SVM train: Starting");
	if( data.rows == 0 ) {

		throw std::logic_error("Empty mat has been submitted");
	}

	computeMaxValues(data);
    normaliseData(data);

    INFO(logger, "Training SVM.")
    SVM.train(data, cv::Mat(1, labels.size(), CV_32SC1, const_cast<int*>(&labels[0])), cv::Mat(), cv::Mat(), params);

    TRACE(logger, "SVM train: Finished");
}

std::vector<float> CvSVMPredictor::predict(const cv::Mat& histogram) {

	assert(histogram.rows == 1);
	INFO(logger, "Classifying")
	TRACE(logger, "predict: Starting");

	cv::Mat data = histogram.clone();
	normaliseData(data);
    std::vector<float> predictions(class_number);
    predictions[SVM.predict(histogram)] = 1;

	TRACE(logger, "predict: Finished");
    return predictions;
}

void CvSVMPredictor::load() {

	TRACE(logger, "load: Starting");
	SVM.load(svmPath.c_str());

	loadVMax();
	TRACE(logger, "load: Finished");
}

void CvSVMPredictor::save() {

	TRACE(logger, "load: Starting");
	SVM.save(svmPath.c_str());
	INFO(logger, "SVM model saved: " + svmPath);

	saveVMax();
	TRACE(logger, "load: Finished");
}

} /* namespace semantic_tagger */


