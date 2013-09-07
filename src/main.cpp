/*
 * Tagger3D : main.cpp
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	
 */

#include "Common/clouds.h"
#include "Common/logger.h"
#include "Config/Config.h"
#include "Common/Tagger3D.h"

#include "log4cxx/consoleappender.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/patternlayout.h"
#include <boost/algorithm/string.hpp>

#include <map>
#include <string>
#include <memory>

namespace t3d = Tagger3D;

void configureLogger(int &argc, char** argv, const lgr::LoggerPtr &logger);

int main(int argc, char** argv) {

	static lgr::LoggerPtr logger(lgr::Logger::getLogger("Main"));
	configureLogger(argc, argv, logger);

	TRACE(logger, "Entering app");

	t3d::Config config(argc, const_cast<const char**>(argv));
	const std::map<std::string, std::string> configMap = config.getConfigMap();

	t3d::Tagger3D tagger(configMap);
	tagger.run();
}

void configureLogger(int &argc, char** argv, const lgr::LoggerPtr &logger) {

	bool foundConfig = false;
	if(argc > 1) {

		std::string filePath;
		std::vector<std::string> splitted;
		for(int i = 1; i < argc; i++) {
			filePath = argv[i];
			boost::split(splitted, filePath, boost::is_any_of("."));
			if(splitted.back() == "ini") {
				foundConfig = true;
				lgr::PropertyConfigurator::configure(filePath);
				break;
			}
		}
	}

	if(!foundConfig) {

		std::string pattern = " %d{HH:mm:ss:SSS} (%c{1}:%L) - %m%n";
		std::string target = "System.out";
		lgr::LayoutPtr layout(new lgr::PatternLayout(pattern));
		lgr::AppenderPtr consoleAppender(new lgr::ConsoleAppender(layout, target));
		consoleAppender->setName("Console");
		logger->addAppender(consoleAppender);
	}
}
