cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project( Tagger3D )

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)
set(CMAKE_ECLIPSE_VERSION 4.3)


# Project details--
set( Project ${CMAKE_PROJECT_NAME} )

set( Tagger3D_Version_Major 1 )
set( Tagger3D_Version_Minor 0 )
set( Tagger3D_Version_Subminor 0 )

# Configure a header file
configure_file( ${CMAKE_SOURCE_DIR}/CMakeScripts/CMakeSettings.h.cmake
		${CMAKE_SOURCE_DIR}/CMakeSettings.h )
	
# Compiler Flags
message( STATUS "Setting GCC flags" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lm ")#-O3 -march=native -mfpmath=sse -funroll-loops -fopenmp" ) 
message( STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}" )

# Use some of our own Find* scripts  
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeScripts" )
message( STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}" )    
   
# PCL    
find_package( PCL 1.7 COMPONENTS common features keypoints REQUIRED ) 
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )  
add_definitions( ${PCL_DEFINITIONS} )
set( LIBS ${LIBS} ${PCL_LIBRARIES} ) 
message( STATUS "PCL Include: ${PCL_INCLUDE_DIRS}" ) 
message( STATUS "PCL Libraries: ${PCL_LIBRARY_DIRS}" )
message( STATUS "PCL Libraries: ${PCL_LIBRARIES}" ) 
 
#	Boost	
set( Boost_USE_MULTITHREADED ON )
find_package( Boost 1.55 COMPONENTS program_options filesystem system REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )
set( LIBS ${LIBS} ${Boost_LIBRARIES} )
message( STATUS "Boost Include: ${Boost_INCLUDE_DIR}" )
message( STATUS "Boost Libraries: ${Boost_LIBRARY_DIRS}" )
message( STATUS "Boost Libraries: ${Boost_LIBRARIES}" ) 

# OpenCV
find_package( OpenCV 2.4.6 COMPONENTS core highgui features2d imgproc REQUIRED)
set( LIBS ${LIBS} ${OpenCV_LIBRARIES} )
message( STATUS "OpenCV Include: ${OpenCV_INCLUDE_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARY_DIRS}" )
message( STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}" )

#	LIBSVM
find_package( LIBSVM REQUIRED )
include_directories( ${LIBSVM_INCLUDE_DIRS})  
set( LIBS ${LIBS} ${LIBSVM_LIBRARIES} )
 
# vlfeat
message(STATUS "vlfeat ${CMAKE_SOURCE_DIR}/../lib/include") 
include_directories("${CMAKE_SOURCE_DIR}/../lib/include")
set(vlfeat_LIBS "${CMAKE_SOURCE_DIR}/../lib/vlfeat/libvl.so") 
set(LIBS ${LIBS} ${vlfeat_LIBS})

#	log4cxx
find_package( log4cxx REQUIRED ) 
include_directories( ${LOG4CXX_INCLUDE_DIRS} )
set( LIBS ${LIBS} ${LOG4CXX_LIBRARIES} )
message( STATUS "LOG4CXX_INCLUDE_DIRS: ${LOG4CXX_INCLUDE_DIRS}" )
message( STATUS "LOG4CXX_LIBRARIES: ${LOG4CXX_LIBRARIES}" )  

FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headers *.h)
SET(dir_list "")
FOREACH(file_path ${headers})
	GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
	SET(dir_list ${dir_list} ${dir_path}) 
ENDFOREACH()
LIST(REMOVE_DUPLICATES dir_list)
include_directories(${dir_list}) 
message( STATUS "Dirs: ${dir_list}" )
 
 # Exec list 
add_executable ( ${Project} ${sources})
	 
message( STATUS "Libs: ${LIBS}" )	 
target_link_libraries ( ${Project} ${LIBS} ) 


