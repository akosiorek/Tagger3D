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

    if (storeHistogram)
        remove( histogramPath.c_str());

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
    labels.push_back(label);


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
	INFO(logger, "Training SVM predictor");
	TRACE(logger, "SVM train: Starting");
	if( imgCount == 0 ) {

		throw std::logic_error("No images has been added");
	}

	cv::Mat normValues = computeMaxValues(DataMat);
    normalizeData(DataMat, normValues);

    SVM.train(DataMat, cv::Mat(1, labels.size(), CV_32SC1, &labels[0]), Mat(), Mat(), params);
    SVM.save(svmPath.c_str());
    saveNormValues(normValues);

    INFO(logger, "SVM model saved: " + svmPath);
    DataMat.release();
    labels.clear();
}


std::vector<int> SVMPredictor::predict() {
	INFO(logger, "Classifying")
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

    confusionMatrix(labels, predictions);
	TRACE(logger, "predict: Finished");
    DataMat.release();
	labels.clear();
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

cv::Mat SVMPredictor::confusionMatrix(const std::vector<int> &labels, const std::vector<int> &predictions) const {

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
		average.at<float>(0, i) = float(confusionMatrix.at<uchar>(i, i)) / classCount.at<int>(0, i);
		avg += confusionMatrix.at<uchar>(i, i);
	}
	avg /= size;


	std::cout << "Entries per class: " << std::endl << classCount << std::endl;
	std::cout << "Confusion Matrix:" << std::endl << confusionMatrix << std::endl;
	std::cout << "Averages: " << std::endl << average << std::endl;
	std::cout << "Average: " << avg << std::endl;


}

} /* namespace semantic_tagger */


