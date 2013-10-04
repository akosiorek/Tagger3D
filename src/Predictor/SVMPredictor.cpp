/*
 * SVMPredictor.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Grzegorz Gwardys
 * Description: svm predictor implementation
 */

#include "SVMPredictor.h"

#include <assert.h>



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

    svmPath = directory + "/" + getParam<std::string>( svmPathKey );
    histogramPath = directory + "/" + getParam<std::string>( histogramPathKey );
    storeHistogram = getParam<bool>(storeHistogramKey);
    dictionarySize = getParam<int>( dictionarySizeKey );

    if (storeHistogram) {
        if( remove( histogramPath.c_str() ) != 0 )   {
        	ERROR(logger, "Cloudn't delete an old histogram file");
        } else
           INFO(logger, "Deleted histogram file");
    }

    createSVM();
    maxValues = Mat::zeros(1, dictionarySize, svmMatType);
}

SVMPredictor::~SVMPredictor() {}

void SVMPredictor::createSVM() {

	params.svm_type    = getParam<int>( svmType );
	params.kernel_type = getParam<int>( kernelType );
	params.term_crit   = cvTermCriteria(
			getParam<int>(termCrit), getParam<int>(maxIter), getParam<double>(epsilon));

	params.gamma = getParam<double>( gamma );
	params.C = getParam<double>( C );
	params.degree = getParam<int>( degree );
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

    if(storeHistogram) {
        std::ofstream out(histogramPath, std::ofstream::app);
        std::copy(histogram.begin(), histogram.end(), std::ostream_iterator<int>(out," "));
        out<<"\n";
        out.close();
    }

    DataMat.push_back(parsedVec);
	imgCount++;
	TRACE(logger, "addImage: Finished");
}

const cv::Mat SVMPredictor::computeMaxValues(const Mat& mat) const{

	TRACE(logger, "computeMaxValues");
	cv::Mat maxValues = mat.row(1).clone();

	int rows = mat.rows;
	int cols = mat.cols;
	int* vmPtr = maxValues.ptr<int>(0);
	for(int row = 0; row < rows; ++row) {
	   const int* matPtr = mat.ptr<int>(row);
	   for(int col = 0; col < cols; ++col)
		   if(matPtr[col] > vmPtr[col])
			   vmPtr[col] = matPtr[col];
	}
	return maxValues;
}

void SVMPredictor::train() {

	TRACE(logger, "SVM train: Starting");
	if( imgCount == 0 ) {

		throw std::logic_error("No images has been added");
	}

	cv::Mat normValues = computeMaxValues(DataMat);
    normalizeData(DataMat, normValues);

    SVM.train(DataMat, labelsMat, Mat(), Mat(), params);
    SVM.save(svmPath.c_str());
    saveNormValues(normValues);

    INFO(logger, "SVM model saved: " + svmPath);
}


std::vector<int> SVMPredictor::predict() {

	TRACE(logger, "predict: Starting");
	if( imgCount == 0 ) {

		throw std::logic_error("No images has been added");
	}

	normalizeData(DataMat, loadNormValues());
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

void SVMPredictor::normalizeData(cv::Mat &mat, const cv::Mat &normValues) const {

    mat.convertTo(mat, svmMatType);
    Mat repeatNormValues = cv::repeat(normValues, mat.rows, 1);
    repeatNormValues.convertTo(repeatNormValues, svmMatType);

    // normalization
    mat = mat / repeatNormValues - Mat::ones(mat.size(), svmMatType);
}

void SVMPredictor::saveNormValues(const cv::Mat &normValues) const {

	std::string path = directory + "/" +normValuesFile;
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	if(!fs.isOpened()) {
		std::runtime_error e("Couldn't open the filestorage at: " + path);
		ERROR(logger, "saveNormValues: " << e.what());
		throw e;
	}

	fs << key << normValues;
	fs.release();
}

const cv::Mat SVMPredictor::loadNormValues() const {

	std::string path = directory + "/" +normValuesFile;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if(!fs.isOpened()) {
		std::runtime_error e("Couldn't open the filestorage at: " + path);
		ERROR(logger, "loadNormValues: " << e.what());
		throw e;
	}

	cv::Mat normValues;
	fs[key] >> normValues;
	fs.release();
	return normValues;
}

} /* namespace semantic_tagger */


