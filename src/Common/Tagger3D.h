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
#include "../ImgReader/ImgReader.h"
#include "../PointNormal/PointNormal.h"
#include "../Detector/Detector.h"
#include "../Descriptor/Descriptor.h"
#include "../Cluster/Cluster.h"
#include "../Predictor/Predictor.h"
#include <memory>

namespace Tagger3D {

/*
 *
 */
class Tagger3D: public ProcessObject {
	/**
	 * Tagger3D is an object categorization.
	 * Provided an RGB-D input it predicts a category of an object.
	 */
public:
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
	int run();

	int trainTest();

private:
	Tagger3D();

	std::unique_ptr<ImgReader> imgReader;
	std::unique_ptr<PointNormal> pointNormal;
	std::unique_ptr<Detector> detector;
	std::unique_ptr<Descriptor> descriptor;
	std::unique_ptr<Cluster> cluster;
	std::unique_ptr<Predictor> predictor;

	enum { TRAIN, TEST, TRAINTEST, PREDICT };
	enum { RANGEIMG_READER, PCD_READER };
	enum { SIFT, ISS3D };
	enum { PFH_DESC, FPFH_DESC };
	enum { SLDA, SVM };

	const std::string loggerName = "Tagger3D";
	const std::string moduleName = "Tagger3D" + separator;

	// Config keys
	const std::string mode = moduleName + "mode";
	const std::string readerType = moduleName + "readerType";
	const std::string detectorType = moduleName + "detectorType";
	const std::string descType = moduleName + "descType";
	const std::string predictorType = moduleName + "predictorType";
	const std::string trainCluster = moduleName + "trainCluster";


};

} /* namespace Tagger3D */
#endif /* TAGGER3D_H_ */
