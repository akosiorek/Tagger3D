/*
 * LibSVMPredictor.h
 *
 *  Created on: August 22, 2013
 *      Author: Adam Kosiorek
 * Description: SVM predictor class declaration
 */

#ifndef SVMPREDICTOR_H_
#define SVMPREDICTOR_H_

#include "Predictor.h"

#include <svm.h>
#include <opencv2/core/core.hpp>

namespace Tagger3D {

/**
 * SVM predictor class. Predicts to which category an image belongs to
 */
class LibSVMPredictor: public Predictor {
public:
	LibSVMPredictor() = delete;

	/**
	 * Parametric constructor
	 * @param _configMap - a map of configuration parameters
	 */
	LibSVMPredictor(const std::map<std::string, std::string> &_configMap,
			const std::string &predictorType = "predictor");

	/**
	 * Default descrutor
	 */
	virtual ~LibSVMPredictor();

	virtual void train(cv::Mat &data, const std::vector<int> &labels) override;
	virtual std::vector<float> predict(const cv::Mat &visualWords) override;
	virtual void load() override;
    virtual void save() override;

private:
    virtual void createSVM();

    // Configuration parameters
    std::string model;
    std::string histogramPath;

    bool storeHistogram;

    //	Configuration keys
    const std::string svmTypeKey = moduleName + "svmType";
    const std::string kernelTypeKey = moduleName + "kernelType";

    const std::string degreeKey = moduleName + "degree";
    const std::string gammaKey = moduleName + "gamma";
    const std::string coef0Key = moduleName + "coef0";
    const std::string epsKey = moduleName + "eps";
    const std::string cache_sizeKey = moduleName + "cacheSize";
    const std::string pKey = moduleName + "p";
    const std::string shrinkingKey = moduleName + "shrinking";
    const std::string probabilityKey = moduleName + "probability";
    const std::string nr_weightKey = moduleName + "nrWeight";
    const std::string CKey = moduleName + "C";
    const std::string modelKey = moduleName + "svmPath";

    svm_parameter params;
    svm_model* svmModel;
};

} /* namespace Tagger3d */
#endif /* SVMPREDICTOR_H_ */
