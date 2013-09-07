/*
 * Tagger3D : Config.cpp
 *
 *  Created on: 	22 lip 2013
 *  Author:			Adam Kosiorek
 *	Description:	Implementation of a config file parser.
 */

#include "Config.h"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/detail/config_file.hpp>

#include <boost/algorithm/string.hpp>

#include <set>
#include <exception>
#include <fstream>

namespace Tagger3D {
namespace pod = boost::program_options::detail;
namespace po = boost::program_options;

Config::Config(const int _argc, const char **_argv) : argc(_argc), argv(_argv) {

	logger = lgr::Logger::getLogger( loggerName );
	DEBUG(logger, "Creating Config");
}

Config::~Config() {

	DEBUG(logger, "Destroying Config");
}

std::map<std::string, std::string> Config::parseConfigFile(std::string filePath) {

	TRACE(logger, "parseConfigFile: Starting");
	std::map<std::string,std::string> confMap;
	std::ifstream config(filePath);

	if(!config)
	{
		std::runtime_error e("Could not open the config file: " + filePath);
		ERROR(logger, "parseConfigFile: " << e.what() );
		throw e;
	}

	//parameters
	std::set<std::string> options;
	std::map<std::string, std::string> parameters;
	options.insert("*");

	try
	{
		for (pod::config_file_iterator i(config, options), e ; i != e; ++i)
		{
			confMap[i->string_key] = i->value[0];
			parameters[i->string_key] = i->value[0];
		}
	}
	catch(std::exception& e)
	{
		std::runtime_error er( e.what() );
		ERROR(logger, "parseConfigFile: " << er.what() );
		throw er;
	}

	TRACE(logger, "parseConfigFile: Finished");
	return confMap;
}

std::map<std::string, std::string> Config::getConfigMap() {

	TRACE(logger, "getConfigMap: Starting");

	std::map<std::string,std::string> confMap;
	std::string helpTmp = help + "," + help[0];
	std::string versionTmp = version + "," + version[0];
	std::string pictureTmp = picture + "," + picture[0];

	/**
	 *	Options allowed only on a command line.
	 */
	po::options_description genericOps("Generic options");
	genericOps.add_options()
			(versionTmp.c_str(), "Print version")
			(helpTmp.c_str(), "Produce help message")
			;

	/**
	 * 	Options allowed on a command line and in a config file.
	 */
	po::options_description dirOps("Directories");
	dirOps.add_options()
		("testImgList", po::value<std::string>(), "A *.txt file with a list of test images");
		("trainImgList", po::value<std::string>(), "A *.txt file with a list of train images");
		("testLabel", po::value<std::string>(), "A *.txt with labels of the testset");
		("trainLabel", po::value<std::string>(), "A *.txt with labels of the trainset");
		;

	po::options_description sldaOps("sLDA options");
	sldaOps.add_options()
		("alpha", po::value<std::string>(), "Alpha value")
		("numTopics", po::value<std::string>(), "Number of topics")
		("initMethod", po::value<std::string>(), "An initialization method")
		("sldaSettings", po::value<std::string>(), "sLDA settings file");
		("sldaOutputDir", po::value<std::string>(), "A sLDA output directory");
		("modelPath", po::value<std::string>(), "A path to a *.model file for sLDA");
		;

	po::options_description clusterOps("Clusterization options");
	clusterOps.add_options()
		("clusterCount", po::value<std::string>(), "Number of clusters")
		("criteriaEps", po::value<std::string>(), "Required precision")
		("criteriaItr", po::value<std::string>(), "Maximum number of iterations")
		;


	po::options_description configOps("Configuration");
	configOps.add_options()
		(pictureTmp.c_str(), po::value<std::string>(), "Path to a picture")
		;
	configOps.add(dirOps).add(sldaOps).add(clusterOps);

	/**
	 *	Hidden options. Allowed on a command line and in a config file.
	 *	These options are concealed from the user.
	 */
	po::options_description hiddenOps("Hidden options");
	hiddenOps.add_options()
		(config.c_str(), po::value<std::string>(), "Configuration file")
		;


	po::options_description cmdOptions;
	cmdOptions.add(genericOps).add(configOps).add(hiddenOps);

	po::options_description visibleOptions("Allowed options:");
	visibleOptions.add(genericOps).add(configOps);

	po::positional_options_description positionalOptions;
	positionalOptions.add(config.c_str(), -1);

	 po::variables_map vm;

	 try{
		 store(po::command_line_parser(argc, argv)
			 .options(cmdOptions).positional(positionalOptions).run(), vm);
	 } catch(std::exception &e) {

		 std::runtime_error er( e.what() );
		 ERROR(logger, "getConfigFile: " << er.what() );
		 throw er;
	 }

	 if( vm.count(help)) {

		 std::cout << visibleOptions;
		 return confMap;
	 }

	 if( vm.count(version) ) {

		 std::cout << "Version xxx";
		 return confMap;
	 }

	 if( !vm.count( config ) || (vm[config].as<std::string>()).size() == 0 ) {

		 std::runtime_error e( "No config file has been specified. Please specify a config file.");
		 ERROR(logger, "getConfigFile: " << e.what() );
		 throw e;
	 }



	std::string filePath = vm[config].as<std::string>();
	confMap = parseConfigFile(filePath);
	for(std::map<std::string, std::string>::iterator it = confMap.begin(); it != confMap.end(); ++it ) {

		std::string key = it->first;
		std::vector<std::string> splitted;
		boost::split(splitted, key, boost::is_any_of("."));
		key = splitted.back();
		if( vm.count( key ) ) {

			it->second = vm[ key ].as< std::string >();
		}

		// Print confMap
		TRACE(logger, it->first << " = " << it->second );
	}

	TRACE(logger, "getConfigMap: Finished");
	return confMap;
}



} /* namespace Tagger3D */
