/*
 * SVMPredictor.h
 *
 *  Created on: August 22, 2013
 *      Author: Grzegorz Gwardys
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
class SVMPredictor: public Predictor {
public:

	/**
	 * Parametric constructor
	 * @param _configMap - a map of configuration parameters
	 */
	SVMPredictor(const std::map<std::string, std::string> &_configMap);

	/**
	 * Default descrutor
	 */
	virtual ~SVMPredictor();

	void addImage(const std::vector<int> &vec, const int &label);
	void train();
    std::vector<int> predict();

private:
	SVMPredictor();
	void createSVM();
	void load();


    Mat labelsMat;
    Mat DataMat;
    //new_val = (old_val-v_min)/(v_max-v_min);
    Mat maxValues;
    Mat minValues;

    void updateMaxValues(const Mat&);

    CvSVMParams params;
    CvSVM SVM;

    // Configuration parameters
    int svmType;
    int kernelType;
    int termCrit;
    std::string svmPath;
    std::string histogramPath;
    bool storeHistogram;
    double epsilon;
    int maxIter;
    double gamma;
    double C;
    int degree;
    int dictionarySize;

    //	Configuration keys
    const std::string svmTypeKey = moduleName + "svmType";
    const std::string kernelTypeKey = moduleName + "kernelType";
    const std::string termCritKey = moduleName + "termCrit";
    const std::string svmPathKey = moduleName + "svmPath";
    const std::string dictionarySizeKey = moduleName + "dictionarySize";
    const std::string histogramPathKey = moduleName + "histogramPath";
    const std::string storeHistogramKey = moduleName + "storeHistogram";
    const std::string epsilonKey = moduleName + "epsilon";
    const std::string maxIterKey = moduleName + "maxIter";
    const std::string degreeKey = moduleName + "degree";
    const std::string gammaKey = moduleName + "gamma";
    const std::string CKey = moduleName + "C";

    const int svmMatType = CV_32F;

    void normalizeData(cv::Mat &mat);
};

} /* namespace semantic_tagger */
#endif /* SVMPREDICTER_H_ */
