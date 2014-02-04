cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( ImgPrep )

# Project details
set( Project ${CMAKE_PROJECT_NAME} )

	
# Compiler Flags
if (UNIX)
	message( STATUS "Setting GCC flags" )
	set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g" )
	#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g" ) 
endif()
message( STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}" )

# Use some of our own Find* scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeScripts" )
message( STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}" ) 
 
# PCL 
find_package( PCL 1.7 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
set( LIBS ${LIBS} ${PCL_LIBRARIES} )
message( STATUS "PCL Include: ${PCL_INCLUDE_DIRS}" )
message( STATUS "PCL Libraries: ${PCL_LIBRARY_DIRS}" )
message( STATUS "PCL Libraries: ${PCL_LIBRARIES}" ) 

#	Boost	
set( Boost_USE_STATIC_LIBS ON )
set( Boost_USE_MULTITHREADED ON )
set( Boost_USE_STATIC_RUNETIME OFF )
set( Boost_NO_SYSTEM_PATH ON )
find_package( Boost 1.55 COMPONENTS program_options filesystem system REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )
set( LIBS ${LIBS} ${Boost_LIBRARIES} )
message( STATUS "Boost Include: ${Boost_INCLUDE_DIR}" )
message( STATUS "Boost Libraries: ${Boost_LIBRARY_DIRS}" )
message( STATUS "Boost Libraries: ${Boost_LIBRARIES}" )

# OpenCV
find_package( OpenCV 2.4.6 COMPONENTS core highgui features2d REQUIRED)
set( LIBS ${LIBS} ${OpenCV_LIBRARIES} )
message( STATUS "OpenCV Include: ${OpenCV_INCLUDE_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARY_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}" )

 
 # Exec list
add_executable ( ${Project} 
	main.cpp
	tinyxml/tinystr.cpp tinyxml/tinyxml.cpp tinyxml/tinyxmlerror.cpp tinyxml/tinyxmlparser.cpp
	)
	 
message( STATUS "Libs: ${LIBS}" )	 
target_link_libraries ( ${Project} ${LIBS} ) 


