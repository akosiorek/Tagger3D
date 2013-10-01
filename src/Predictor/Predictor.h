/*
 * Predictor.h
 *
 *  	Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 *  	Description: Predictor base class declaration
 */

#ifndef PREDICTER_H_
#define PREDICTER_H_

#include "../Common/ProcessObject.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>

using namespace cv;
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
	Predictor(const std::map<std::string, std::string> &_configMap);

	/**
	 * Default descrutors
	 */
	virtual ~Predictor();

	/**
	 * Adds a vector of visual words to predictor's internal container.
	 * A default label is assigned to each vector.
	 * @param vec	a vector of visual words
	 * @return	void
	 */
	virtual void addImage(const std::vector<int> &vec);

	/**
	 * Adds a vector of visual words to predictor's internal container.
	 * A specified label is assigned to each vector
	 * @param	vec	a vector of visual words
	 * @param	label a speficied label
	 * @return	void
	 */
	virtual void addImage(const std::vector<int> &vec, const int &label) = 0;

	/**
	 * Trains the predictor using all the vector of visual words/descriptors in the internal storage
	 * @param	none
	 * @return	void
	 */
	virtual void train() = 0;

	/**
	 * Trains the predictor using specified vectors of visual words and their labels
	 * @param	vecs a vector of vectors of visual words
	 * @param	labels a vector of labels
	 */
	virtual void train(const std::vector<std::vector<int>> &vecs, const std::vector<int> &labels);

	/**
	 * Predicts classes of all images from the internal storage
	 * @param	none
	 * @return	void
	 */
	virtual std::vector<int> predict() = 0;

	/**
	 * Predics a class of an input image
	 * @param	visualWords	a vec of visual words describing a single image
	 * @return	void
	 */
	virtual std::vector<int> predict(const std::vector<int> &visualWords);

	/**
	 * Predics categories for a bach of images in order to evaluate an average accuracy
	 * @param	vecs a vector of vectors of visual words
	 * @param	labels a vector of labels
	 * @return	void
	 */
	virtual void predict(const std::vector<std::vector<int>> &vecs, const std::vector<int> &labels);

protected:

	/**
	 * Name of the module in a config map
	 */
	const std::string moduleName = "predictor" + separator;
	bool modelLoaded;
	int imgCount;

private:
	Predictor();

	/**
	 * Name of the logger
	 */
	const std::string loggerName = "Main.Predictor";
	const std::string defaultLabelKey = moduleName + "default_label";

	int defaultLabel;
};

} /* namespace semantic_tagger */
#endif /* PREDICTER_H_ */
