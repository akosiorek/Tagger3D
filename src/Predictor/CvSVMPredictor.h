/*
 * CvSVMPredictor.h
 *
 *  Created on: August 22, 2013
 *      Author: Adam Kosiorek
 * Description: SVM predictor class declaration
 */

#ifndef SVMPREDICTER_H_
#define SVMPREDICTER_H_

#include "Predictor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


namespace Tagger3D {

/**
 * SVM predictor class. Predicts to which category an image belongs to
 */
class CvSVMPredictor: public Predictor {
public:

	CvSVMPredictor();
	/**
	 * Parametric constructor
	 * @param _configMap - a map of configuration parameters
	 */
	CvSVMPredictor(const std::map<std::string, std::string> &_configMap,
			const std::string &predictorType = "predictor");

	/**
	 * Default descrutor
	 */
	virtual ~CvSVMPredictor();

	virtual void train(cv::Mat &data, const std::vector<int> &labels) override;
	virtual std::vector<float> predict(const cv::Mat &visualWords) override;
	virtual void load() override;
	virtual void save() override;

private:
	virtual void createSVM();

    CvSVMParams params;
    CvSVM SVM;

    // Configuration parameters
    std::string svmPath;
    std::string histogramPath;
    bool storeHistogram;
    int dictionarySize;

    //	Configuration keys
    const std::string svmType = moduleName + "svmType";
    const std::string kernelType = moduleName + "kernelType";
    const std::string termCrit = moduleName + "termCrit";
    const std::string svmPathKey = moduleName + "svmPath";
    const std::string dictionarySizeKey = "dictionarySize";
    const std::string histogramPathKey = moduleName + "histogramPath";
    const std::string storeHistogramKey = moduleName + "storeHistogram";
    const std::string epsilon = moduleName + "epsilon";
    const std::string maxIter = moduleName + "maxIter";
    const std::string degree = moduleName + "degree";
    const std::string gamma = moduleName + "gamma";
    const std::string C = moduleName + "C";

    const int svmMatType = CV_32F;

    const std::string normValuesFile = "maxValues.xml";
    const std::string key = "key";


};

} /* namespace semantic_tagger */
#endif /* SVMPREDICTER_H_ */
