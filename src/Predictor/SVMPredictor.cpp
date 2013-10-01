/*
 * SVMPredictor.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Grzegorz Gwardys
 * Description: svm predictor implementation
 */

#include "SVMPredictor.h"

#include <assert.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
#include<fstream>
#include<iterator>

#include<stdio.h>

namespace Tagger3D {

SVMPredictor::SVMPredictor(const std::map<std::string, std::string> &configMap) : Predictor(configMap) {

    svmType = getParam<int>( svmTypeKey );
    kernelType = getParam<int>( kernelTypeKey );
    termCrit  = getParam<int>( termCritKey );
    svmPath = directory + "/" + getParam<std::string>( svmPathKey );
    histogramPath = directory + "/" + getParam<std::string>( histogramPathKey );
    storeHistogram = getParam<bool>(storeHistogramKey);
    dictionarySize = getParam<int>( dictionarySizeKey );
    epsilon = getParam<double>( epsilonKey );
    maxIter = getParam<int>( maxIterKey );
    gamma = getParam<double>( gammaKey );
    C = getParam<double>( CKey );
    degree = getParam<int>( epsilonKey );

    if (storeHistogram) {//if store, than delete old-file
        if( remove( histogramPath.c_str() ) != 0 ){
           ERROR(logger, "Cloudn't delete the old histogram file");
        }
        else
           INFO(logger, "Deleted histogram file");
    }

    createSVM();
    maxValues = Mat::zeros(1, dictionarySize, svmMatType);
}

SVMPredictor::~SVMPredictor() {}

void SVMPredictor::createSVM() {

	params.svm_type    = svmType;
	params.kernel_type = kernelType;
	params.term_crit   = cvTermCriteria(termCrit, maxIter, epsilon);

	//best parameters from cross-validation
	params.gamma = gamma;  // for poly/rbf/sigmoid
	//params.gamma = 3.05176e-05;
	params.C = C;

	params.degree = degree;
	//params.class_weights=NULL;
}

void SVMPredictor::addImage(const std::vector<int> &vec, const int &label) {

    TRACE(logger, "addImage: adding image with label: " << label);
    labelsMat.push_back(label);
    //make histogram
    std::vector<int> histogram (dictionarySize, 0);
    int* histPtr = &histogram[0];
    for (const auto &el: vec)
       histPtr[el - 1] += 1;
    Mat parsedVec = Mat(1, dictionarySize, CV_32S, &histogram[0]);

    if(storeHistogram){     //TODO: another function, (maybe: change filename at training/prediction stage)
        std::ofstream out(histogramPath, std::ofstream::app);
        std::copy(histogram.begin(),histogram.end(),std::ostream_iterator<int>(out," "));
        out<<"\n";
        out.close();
    }

    updateMaxValues(parsedVec);
    DataMat.push_back(parsedVec);
	imgCount++;
	TRACE(logger, "addImage: Finished");
}


void SVMPredictor::updateMaxValues(const Mat& vec){

	TRACE(logger, "updateMaxValues");
	int rows = vec.rows;
	int cols = vec.cols;

	for(int row = 0; row < rows; ++row) {
	   int* vmPtr = maxValues.ptr<int>(row);
	   const int* vecPtr = vec.ptr<int>(row);
	   for(int col = 0; col < cols; ++col)
		   if(vecPtr[col] > vmPtr[col])
			   vmPtr[col] = vecPtr[col];
   }
}

void SVMPredictor::train() {

	TRACE(logger, "SVM train: Starting");
	if( imgCount == 0 ) {

		throw std::logic_error("No images has been added");
	}

    normalizeData(DataMat);

    // Train the SVM
    SVM.train(DataMat, labelsMat, Mat(), Mat(), params);
    SVM.save(svmPath.c_str());

    INFO(logger, "SVM model saved: " + svmPath);
}


std::vector<int> SVMPredictor::predict() {

	TRACE(logger, "predict: Starting");
	if( imgCount == 0 ) {

		throw std::logic_error("No images has been added");
	}
	normalizeData(DataMat);
    std::vector<int> predictions;
    predictions.reserve(DataMat.rows);
    SVM.load(svmPath.c_str());
    for(int row = 0; row < DataMat.rows; ++row)
        predictions.push_back(SVM.predict(DataMat.row(row)));

    Mat predictionsMat = Mat(predictions);
    Mat results = abs(predictionsMat-labelsMat);
    results.convertTo(results, CV_8UC1);
    Mat locations;
    cv::findNonZero(results, locations);
    std::cout<<"avg: "<<100*float(results.size().height-locations.size().height)/results.size().height<<"%"<<std::endl;
	TRACE(logger, "predict: Finished");
    return predictions;
}

void SVMPredictor::load() {

	TRACE(logger, "load: Starting");
	

	TRACE(logger, "load: Finished");
}

void SVMPredictor::normalizeData(cv::Mat &mat) {

    mat.convertTo(mat, svmMatType); // float conversion
    Mat repeatMaxValues;
    repeat(maxValues, mat.rows, 1, repeatMaxValues );
    repeatMaxValues.convertTo(repeatMaxValues, svmMatType);

    // normalization
    mat = mat / repeatMaxValues - Mat::ones(mat.size(), svmMatType);
}

} /* namespace semantic_tagger */


