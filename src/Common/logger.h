/*
 * logger.h
 *
 *  		Created on: 24 Jul 2013
 *      	Author: Adam Kosiorek
 *      	Description:	The header contains macros simplyfing the usage of LOG4CXX
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <log4cxx/logger.h>

#define TRACE(a, b) LOG4CXX_TRACE(a, b)
#define DEBUG(a, b) LOG4CXX_DEBUG(a, b)
#define INFO(a, b) LOG4CXX_INFO(a, b)
#define WARN(a, b) LOG4CXX_WARN(a, b)
#define ERROR(a, b) LOG4CXX_ERROR(a, b)
#define FATAL(a, b) LOG4CXX_FATAL(a, b)

namespace lgr = log4cxx;

#endif /* LOGGER_H_ */
