/*
 * SldaPredictor.h
 *
 *  Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 * Description: sLDA predictor class declaration
 */

#ifndef SLDAPREDICTER_H_
#define SLDAPREDICTER_H_

#include "Predictor.h"

#include "slda/corpus.h"
#include "slda/utils.h"
#include "slda/slda.h"

#include <memory>

namespace Tagger3D {

/**
 * sLDA predictor class. Predicts to which category an image belongs to
 */
class SldaPredictor: public Predictor {
public:

	/**
	 * Parametric constructor
	 * @param _configMap - a map of configuration parameters
	 */
	SldaPredictor(const std::map<std::string, std::string> &_configMap);

	/**
	 * Default descrutor
	 */
	virtual ~SldaPredictor();

	void addImage(const std::vector<int> &vec, const int &label);
	void train();
	std::vector<int> predict();

private:
	SldaPredictor();
	void createSlda();
	void load();

	std::unique_ptr<settings> sldaSet;
	std::unique_ptr<corpus> sldaCorpus;
	std::unique_ptr<slda>	sldaModel;

	// Configuration parameters
	std::string sldaSettings;
	std::string sldaInitMethod;
	std::string sldaModelDir;
	float alpha;
	int numTopics;

	//	Configuration keys
	const std::string sldaSettingsKey = moduleName + "settings";
	const std::string sldaInitMethodKey = moduleName + "init";
	const std::string sldaModelDirKey = moduleName + "model";
	const std::string alphaKey = moduleName + "alpha";
	const std::string numTopicsKey = moduleName + "num_topics";
	const std::string settingsDir = moduleName + "settingsDir";



};

} /* namespace semantic_tagger */
#endif /* SldaPREDICTER_H_ */
