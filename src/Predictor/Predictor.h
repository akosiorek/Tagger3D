/*
 * Predictor.h
 *
 *  	Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 *  	Description: Predictor base class declaration
 */

#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include "ProcessObject.h"
#include "IoUtils.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <memory>

namespace Tagger3D {

/**
 * Base class for predictors.
 */
class Predictor : public ProcessObject {
public:

	Predictor() = delete;
	/**
	 * Parametric constructor
	 * @param _configMap	a key-value map of configuration parameters
	 */
	Predictor(const std::map<std::string, std::string> &_configMap,	const std::string predictorType);

	/**
	 * Default descrutors
	 */
	virtual ~Predictor();

	/**
	 * Trains the predictor using specified vectors of visual words and their labels
	 * @param	vecs a vector of vectors of visual words
	 * @param	labels a vector of labels
	 */
	virtual void train(cv::Mat &data, const std::vector<int> &labels) = 0;

	/**
	 * Predics a class of an input image
	 * @param	visualWords	a vec of visual words describing a single image
	 * @return	void
	 */
    virtual std::vector<float> predict(const cv::Mat &visualWords) = 0;

    virtual void load() = 0;
    virtual void save() = 0;

protected:

	virtual void createSVM() = 0;
	virtual void normaliseData(cv::Mat &mat);
	virtual const cv::Mat computeMaxValues(const cv::Mat& mat) const;
	cv::Mat confusionMatrix(const std::vector<int> &labels, const std::vector<int> &predictions) const;
	void saveVMax();
	void loadVMax();

	/**
	 * Name of the module in a config map
	 */
	const std::string moduleName;
    std::shared_ptr<IoUtils> io;

    int class_number;
    int dims;
    std::string normalizationPath;

private:

	/**
	 * Name of the logger
	 */
	const std::string loggerName = "Main.Predictor";
	const std::string class_numberKey = moduleName + "classes";
	const std::string normalizationPathKey = moduleName + "normalizationPath";



	 const int REDUCE_TO_ROW = 0;
	 cv::Mat v_max;
};

} /* namespace Tagger3D */
#endif /* PREDICTOR_H_ */
