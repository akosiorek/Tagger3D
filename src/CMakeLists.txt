cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( Tagger3D )

# Project details
set( Project ${CMAKE_PROJECT_NAME} )

set( Tagger3D_Version_Major 1 )
set( Tagger3D_Version_Minor 0 )
set( Tagger3D_Version_Subminor 0 )

# Configure a header file
configure_file( ${CMAKE_SOURCE_DIR}/CMakeScripts/CMakeSettings.h.cmake
		${CMAKE_SOURCE_DIR}/CMakeSettings.h )
	
# Compiler Flags
if (UNIX)
	message( STATUS "Setting GCC flags" )
	set( CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -L/usr/local/lib -lgsl -lgslcblas -lm" )
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
find_package( OpenCV 2.4.6 COMPONENTS core highgui features2d imgproc ml REQUIRED)
set( LIBS ${LIBS} ${OpenCV_LIBRARIES} )
message( STATUS "OpenCV Include: ${OpenCV_INCLUDE_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARY_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}" )


#	GSL
find_package( GSL REQUIRED )
include_directories( ${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS} )
set( LIBS ${LIBS} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES} )


#	log4cxx
find_package( log4cxx REQUIRED )
include_directories( ${LOG4CXX_INCLUDE_DIRS} )
set( LIBS ${LIBS} ${LOG4CXX_LIBRARIES} )
message( STATUS "LOG4CXX_INCLUDE_DIRS: ${LOG4CXX_INCLUDE_DIRS}" )
message( STATUS "LOG4CXX_LIBRARIES: ${LOG4CXX_LIBRARIES}" )  

 
 # Exec list
add_executable ( ${Project} 
	main.cpp 
	Common/ProcessObject.cpp Common/Tagger3D.cpp
	Config/Config.cpp  
	ImgReader/ImgReader.cpp ImgReader/RangeImgReader.cpp ImgReader/PcdReader.cpp
	PointNormal/PointNormal.cpp PointNormal/NormalEstimator.cpp 
	Detector/Detector.cpp Detector/SIFTDetector.cpp
	Descriptor/Descriptor.cpp Descriptor/PFHDescriptor.cpp Descriptor/FPFHDescriptor.cpp
	Descriptor/CloudParser/CloudParser.cpp 
	Cluster/Cluster.cpp Cluster/KMeansCluster.cpp
	Predictor/Predictor.cpp Predictor/SldaPredictor.cpp Predictor/SVMPredictor.cpp
	Predictor/svm/svm.cpp Predictor/svm/svm-predict.c Predictor/svm/svm-train.c
	Predictor/slda/corpus.cpp Predictor/slda/opt.cpp Predictor/slda/slda.cpp Predictor/slda/utils.cpp
	)
	 
message( STATUS "Libs: ${LIBS}" )	 
target_link_libraries ( ${Project} ${LIBS} ) 


