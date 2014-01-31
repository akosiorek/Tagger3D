/*
 * LibSVMPredictor.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Adam Kosiorek
 * Description: svm predictor implementation
 */

#include "LibSVMPredictor.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

namespace Tagger3D {

LibSVMPredictor::LibSVMPredictor(const std::map<std::string, std::string> &_configMap,
		const std::string &predictorType)

	: Predictor(_configMap, predictorType),
	svmModel(nullptr) {

	model = io->getPath() + "/" + getParam<std::string>(modelKey);
	createSVM();
}

LibSVMPredictor::~LibSVMPredictor() {

	svm_free_and_destroy_model(&svmModel);
	svm_destroy_param(&params);
}

void LibSVMPredictor::createSVM() {

    params.svm_type = getParam<int>( svmTypeKey );
    params.kernel_type  = getParam<int>( kernelTypeKey );
    params.degree = NULL;//getParam<int>( degreeKey );
    params.gamma = getParam<double>( gammaKey );
    params.coef0 = NULL;//getParam<int>( coef0Key );
    params.eps = getParam<double>( epsKey );
    params.cache_size = getParam<int>( cache_sizeKey );
//    params.p = NULL;//getParam<double>( pKey );
    params.shrinking = getParam<int>( shrinkingKey );
    params.probability = getParam<int> (probabilityKey);
    params.nr_weight = getParam<int> (nr_weightKey);
    params.C = getParam<double>( CKey );
    params.weight_label = NULL;
    params.weight = NULL;

}

void LibSVMPredictor::train( cv::Mat &data, const std::vector<int> &labels) {
	INFO(logger, "Training SVM");
    TRACE(logger, "SVM train: Starting");

	if( data.empty()) {
		std::logic_error e("Cannot train the SVM without any data");
		ERROR(logger, "train: " << e.what());
		throw e;
	}

	int rows = data.rows;
	int cols = data.cols;

	computeMaxValues(data);
    normaliseData(data);

    svm_problem prob;
    prob.l =  rows;
    prob.x  = (struct svm_node**)malloc(rows * sizeof(struct svm_node*));
	for(int r = 0; r < rows; ++r) {
		prob.x[r] = (struct svm_node*)malloc((cols + 1) * sizeof(struct svm_node));
	}

	prob.y = (double*)malloc(rows * sizeof(double));
	const int* labPtr = &labels[0];
	for (int r = 0; r < rows; ++r) {
		prob.y[r] = (double)labPtr[r];
	}

    for (int r = 0; r < rows; ++r) {
    	float *dataPtr = data.ptr<float>(r); // Get data from OpenCV Mat
    	long counter = 0;
        for (int c = 0; c < cols; ++c) {
        	if(dataPtr[c] != -1) {
				prob.x[r][counter].index = c + 1;  // Index starts from 1; Pre-computed kernel starts from 0
				prob.x[r][counter].value = (double)dataPtr[c];
				counter++;
        	}
        }
        prob.x[r][counter].index = -1;   // End of line
    }

    // Train
    svmModel = svm_train(&prob, &params);

    //	Clean up
    for(size_t r = 0; r < rows; ++r) {
        free(prob.x[r]);
    }
    free(prob.x);
    free(prob.y);
    TRACE(logger, "SVM train: Finished");
}

std::vector<float> LibSVMPredictor::predict(const cv::Mat &histogram) {

	assert(histogram.rows == 1);
	cv::Mat visualWords = histogram.clone();
	normaliseData(visualWords);

	std::vector<double> predictions(class_number);
	svm_node *svmVec = (struct svm_node *)malloc((dims + 1) * sizeof(struct svm_node));
	const float* hPtr = visualWords.ptr<float>();
	int counter = 0;
	for(int i = 0; i < dims; ++i) {
		if(hPtr[i] != -1) {
		svmVec[counter].index = i + 1;
		svmVec[counter].value = (double)hPtr[i];
		counter++;
		}
	}
	svmVec[counter].index = -1;
	svm_predict_probability(svmModel, svmVec, &predictions[0]);

	return std::vector<float>(predictions.begin(), predictions.end());
}

void LibSVMPredictor::load() {
	INFO(logger, "Loading SVM");
	TRACE(logger, "load: Starting");
	if((svmModel = svm_load_model(model.c_str())) == 0) {
		std::runtime_error e("Could not load the SVM model from " + model);
		ERROR(logger, e.what());
		throw e;
	}

	loadVMax();
	TRACE(logger, "load: Finished");
}

void LibSVMPredictor::save() {
	TRACE(logger, "save: Starting")
	svm_save_model(model.c_str(), svmModel);

	saveVMax();
	INFO(logger, "SVM model saved: " + model);
	TRACE(logger, "save: Finished")
}

} /* namespace Tagger3D */
