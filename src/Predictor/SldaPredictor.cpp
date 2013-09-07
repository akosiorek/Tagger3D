/*
 * SldaPredictor.cpp
 *
 *  Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 * Description: sLda predictor implementation
 */

#include "SldaPredictor.h"

#include <assert.h>

namespace Tagger3D {

SldaPredictor::SldaPredictor(const std::map<std::string, std::string> &_configMap) : Predictor(_configMap) {


	sldaSettings = const_cast<char*>(getParam<std::string>( sldaSettingsKey ).c_str() );

	if( !fileExists(sldaSettings) ) {
		std::runtime_error e("File " + sldaSettings + " does not exist");
		ERROR(logger, e.what() );
		throw e;
	}

	sldaInitMethod = const_cast<char*>(getParam<std::string>( sldaInitMethodKey ).c_str() );
	sldaDirectory = const_cast<char*>(getParam<std::string>( sldaDirectoryKey ).c_str() );
	sldaModelDir = const_cast<char*>(getParam<std::string>( sldaModelDirKey).c_str() );
	alpha = getParam<float>( alphaKey );
	numTopics = getParam<int>( numTopicsKey );
	createSlda();
	assert( sldaCorpus != nullptr );
	assert( sldaSet != nullptr );
	assert( sldaModel != nullptr );

}

SldaPredictor::~SldaPredictor() {}

void SldaPredictor::createSlda() {

	sldaCorpus = std::unique_ptr<corpus>( new corpus() );
	sldaSet = std::unique_ptr<settings>( new settings() );
	sldaModel = std::unique_ptr<slda>( new slda() );

	sldaSet->read_settings( const_cast<char*>( sldaSettings.c_str()) );
}

void SldaPredictor::addImage(const std::vector<int> &vec, const int &label) {

	TRACE(logger, "addImage: adding image with label: " << label);
	sldaCorpus->read_vector( vec, label );
	imgCount++;
}

void SldaPredictor::train() {

	TRACE(logger, "train: Starting");
	if( imgCount == 0 ) {

		std::invalid_argument e("No image has been added");
		ERROR(logger, "train: " << e.what() );
		throw e;
	}

	make_directory( const_cast<char*>( sldaDirectory.c_str()) );
	sldaModel->init(alpha, numTopics, sldaCorpus.get() );
	sldaModel->v_em( sldaCorpus.get(), sldaSet.get(), const_cast<char*>( sldaInitMethod.c_str()), const_cast<char*>( sldaDirectory.c_str()) );
	TRACE(logger, "train: Finished");
}

std::vector<int> SldaPredictor::predict() {

	TRACE(logger, "predict: Starting");
	if( imgCount == 0 ) {
		std::logic_error e("No images has been added");
		ERROR(logger, "predict: " << e.what());
		throw e;
	}

	if( imgCount > 1) {
		sldaModel->showResults = false;
		sldaModel->showAverage = true;
	} else {
		sldaModel->showResults = true;
		sldaModel->showAverage = false;
	}

	if( !modelLoaded )
		load();

	make_directory( const_cast<char*>( sldaDirectory.c_str()) );
	sldaModel->infer_only( sldaCorpus.get(), sldaSet.get(), const_cast<char*>( sldaDirectory.c_str()) );
	TRACE(logger, "predict: Finished");
	//TODO return something useful
	return std::vector<int>();
}

void SldaPredictor::load() {

	TRACE(logger, "load: Starting");
	sldaModel->load_model(const_cast<char*>( sldaModelDir.c_str()));
	TRACE(logger, "load: Finished");
}

} /* namespace semantic_tagger */
