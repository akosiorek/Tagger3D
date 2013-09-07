/*
 * Predictor.cpp
 *
 *  Created on: Jul 4, 2013
 *      Author: Adam Kosiorek
 * Description: Predictor base class implementation
 */

#include "Predictor.h"

namespace Tagger3D {

Predictor::Predictor(const std::map<std::string, std::string> &_configMap)
	: ProcessObject(_configMap) {


	logger = lgr::Logger::getLogger(loggerName);
	DEBUG(logger, "Creating Predictor");

	if( _configMap.empty() ) {

		throw std::invalid_argument("Empty configuration map");
	}
	defaultLabel = getParam<int>(defaultLabelKey);
	modelLoaded = false;
	imgCount = 0;
}

Predictor::~Predictor() {

	TRACE(logger, "Destroying Predictor");
}

void Predictor::train(const std::vector<std::vector<int>> &vecs, const std::vector<int> &labels) {

	if( vecs.size() != labels.size() ) {

		std::invalid_argument e("Labels' count and vectors' count differ");
		ERROR(logger, "train: " << e.what() );
		ERROR(logger, "labels size = " << labels.size() << " vecs size = " << vecs.size() );
		throw e;
	}


	for(int i = 0; i < vecs.size(); ++i) {

		addImage( vecs[i], labels[i] );
	}
	train();
}

std::vector<int> Predictor::predict(const std::vector<int> &visualWords) {

	addImage(visualWords);
	predict();
	//TODO return something useful
	return std::vector<int>();
}

void Predictor::predict(const std::vector<std::vector<int>> &vecs, const std::vector<int> &labels) {

	int size = vecs.size();
	if( size != labels.size() ) {

		std::invalid_argument e("Labels' count and vectors' count differ");
		ERROR(logger, "train: " << e.what() );
		ERROR(logger, "labels size = " << labels.size() << " vecs size = " << vecs.size() );
		throw e;
	}

	for(int i = 0; i < size; ++i)
		addImage( vecs[i], labels[i] );
	predict();
}

void Predictor::addImage(const std::vector<int> &vec) {

	addImage(vec, defaultLabel);
}

} /* namespace semantic_tagger */
