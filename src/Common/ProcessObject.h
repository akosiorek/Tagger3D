/*
 * Tagger3D : ProcessObject.h
 *
 *  Created on: 	27 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#ifndef PROCESSOBJECT_H_
#define PROCESSOBJECT_H_

#include "logger.h"

#include <string>
#include <map>
#include <vector>

#include <sstream>
#include <typeinfo>
#include <stdio.h>
#include <stdexcept>
#include <iostream>

namespace Tagger3D {

class ProcessObject {
public:
	ProcessObject(const std::map<std::string, std::string> &_configMap);
	virtual ~ProcessObject();

protected:
	template<typename T>
	T getParam(const std::string &key);
	std::string getParam(const std::string &key);
	bool fileExists(const std::string &path);

	const std::string separator = ".";
	lgr::LoggerPtr logger;
	std::string directory;
	const std::string directoryKey = "directory";

private:
	ProcessObject();
	bool checkConfig(const std::string &key);
	bool checkConfig(const std::vector<std::string*> keys);
	template<typename T>
	T stringToNumber(const std::string &s, T def = T() );

	std::map<std::string, std::string> configMap;
};

template<typename T>
T ProcessObject::getParam(const std::string &key) {

	if( !checkConfig( key )) {

		throw std::runtime_error("No parameter " + key + " in the configMap");
	}

	std::string value = configMap.find(key)->second;
	return stringToNumber<T>(value);
}

template<typename T>
T ProcessObject::stringToNumber(const std::string &s, T def) {

	std::stringstream ss(s);
	T result;
	return ss >> result ? result : def;
}

} /* namespace Tagger3D */
#endif /* PROCESSOBJECT_H_ */


