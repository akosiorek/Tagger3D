/*
 * Tagger3D : ProcessObject.cpp
 *
 *  Created on: 	27 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "ProcessObject.h"
#include <fstream>

namespace Tagger3D {

ProcessObject::ProcessObject(const std::map<std::string, std::string> &_configMap) : configMap(_configMap) {

	if( _configMap.empty() ) {

		throw std::invalid_argument("Empty configuration map");
	}
}

ProcessObject::~ProcessObject() {}

bool ProcessObject::checkConfig(const std::string &key) {

	if( !configMap.count(key) || configMap.find(key)->second.length() == 0) {

		return false;
	}

	return true;
}

bool ProcessObject::checkConfig(const std::vector<std::string*> keys) {

	for(std::string* key : keys) {

		if( !checkConfig( *key) ) {

			return false;
		}

		return true;
	}
}

std::string ProcessObject::getParam(const std::string &key ) {

	if( !checkConfig( key )) {

		throw std::runtime_error("No parameter " + key + " in the configMap");
	}

	return configMap.find(key)->second;
}

bool ProcessObject::fileExists(const std::string &path) {

	try {
		std::ifstream is(path);
		if( is.good() ) {

			is.close();
			return true;
		}
		return false;
	} catch (std::exception &e) {

		return false;
	}
}



} /* namespace Tagger3D */
