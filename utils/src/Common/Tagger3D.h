/*
 * Tagger3D.h
 *
 *   Created on: 23 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef TAGGER3D_H_
#define TAGGER3D_H_

#include "ProcessObject.h"
#include "ImgReader.h"
#include "PointNormal.h"
#include "Detector.h"
#include "Descriptor.h"
#include "Cluster.h"
#include "Predictor.h"
#include "IoUtils.h"
#include <memory>

namespace Tagger3D {

/**
 * Tagger3D is an object categorization.
 * Provided an RGB-D input it predicts a category of an object.
 */
class Tagger3D: public ProcessObject {

	//	-------------------------------------------------------------------------
	//	Methods	------------------------------------------------------------------
public:
	Tagger3D() = delete;
	Tagger3D(const std::map<std::string, std::string> &configMap);
	virtual ~Tagger3D();

	/**
	 * Performs model training
	 * @param	void
	 * @return	void
	 */
	void train();

	/**
	 * Performs batch prediction in order to evaluate an average accuracy
	 * @param void
	 * @return void
	 */
	void test();

	/**
	 * Predicts a category membership of a provided tuple of rgb and depth images
	 * @param	rgbPath	path to an rgb image
	 * @param	depthPath	path to a depth image
	 * @return	an int value being a category number
	 */
	int predict(const std::string &rgbPath, const std::string &depthPath);

	/**
	 * Launched one of the train, test or predict mode based on the config settings
	 * @param none
	 * @return an int number of a category in case the prediction mode was launched
	 */
	void run();

	//	Separate run options ------------------------------------------------
	void descRun();
	void clustRun();
	void trainRun();
	void predRun();

private:
	std::vector<cv::Mat> computeDescriptors();
	int getRunMode();
	int getCatNum(const std::vector<float> &vec) const;

	//	-----------------------------------------------------------------------
	// Fields	----------------------------------------------------------------
public:
private:
	// Pointers	----------------------------------------------------------------
	std::unique_ptr<ImgReader> imgReader;
	std::unique_ptr<PointNormal> pointNormal;
	std::unique_ptr<Detector> detector;
	std::unique_ptr<Descriptor> descriptor;
	std::unique_ptr<Cluster> cluster;
	std::unique_ptr<Predictor> predictor;
	std::shared_ptr<IoUtils> io;

	// Constants	----------------------------------------------------------------
	const std::string loggerName = "Tagger3D";
	const std::string moduleName = "Tagger3D" + separator;

	// Config keys	---------------------------------------------------------------
	const std::string trainDescName = "trainDescriptors";
	const std::string testDescName = "testDescriptors";
	const std::string trainHistogram = "trainHistogram";
	const std::string testHistogram = "testHistogram";

	const std::string storeHistogramKey = "storeHistogram";

	//	Modes	-------------------------------------------------------------------
	const std::vector<std::string> modeStrings = {"desc", "clust", "train", "pred"};

	// Enums -------------------------------------------------------------------
	enum Mode { DESC, CLUST, TRAIN, PRED };
};

} /* namespace Tagger3D */
#endif /* TAGGER3D_H_ */
