/*
 * Tagger3D : Config.h
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	A config file parser.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "../Common/logger.h"

#include <boost/program_options/variables_map.hpp>

namespace Tagger3D {

class Config {
public:

	Config(const int _argc, const char **_argv);
	virtual ~Config();

	/**
	 *	Prepares and returns a config map.
	 *	@retun	std::map<std::string, std::string> containing pairs of key=valuegetConfigMap
	 */
	std::map<std::string,std::string> getConfigMap();

private:

	int argc;
	const char** argv;

	const std::string config = "config";
	const std::string help = "help";
	const std::string version = "version";
	const std::string picture = "picture";

	std::map<std::string,std::string> parseConfigFile(std::string filePath);

	const std::string loggerName = "Main.Config";
	lgr::LoggerPtr logger;
};

} /* namespace Tagger3D */
#endif /* CONFIG_H_ */
