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

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>

namespace Tagger3D {

/**
 * Base class for predictors.
 */
class Predictor : public ProcessObject {
public:

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

	/**
	 * Name of the module in a config map
	 */
	const std::string moduleName;

private:
	Predictor();

	/**
	 * Name of the logger
	 */
	const std::string loggerName = "Main.Predictor";
};

} /* namespace Tagger3D */
#endif /* PREDICTOR_H_ */
