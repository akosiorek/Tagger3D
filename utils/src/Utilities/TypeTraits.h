/*
 * CvTypeTraits.h
 *
 *  Created on: 4 Oct 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#ifndef CVTYPETRAITS_H_
#define CVTYPETRAITS_H_

#include <opencv2/core/types_c.h>
#include <opencv2/core/core.hpp>
#include <stdexcept>

//	CvTypeTraits -------------------------------------------------------------------------------------

template<typename T> struct CvTypeTraits {
static int type() { throw std::logic_error("Unimplemented"); }
};

template<> struct CvTypeTraits<uchar> {
	static int type() {	return CV_8UC1; }
};

template<> struct CvTypeTraits<ushort> {
	static int type() {	return CV_16UC1; }
};

template<> struct CvTypeTraits<int> {
	static int type() {	return CV_32SC1; }
};

template<> struct CvTypeTraits<float> {
	static int type() {	return CV_32FC1; }
};

template<> struct CvTypeTraits<double> {
	static int type() {	return CV_64FC1; }
};


//	CvMatTraits -------------------------------------------------------------------------------------


template<int i> struct CvMatTraits {
	typedef int type;
};

template<> struct CvMatTraits<CV_8UC1> {
	typedef uchar type;
};

template<> struct CvMatTraits<CV_16UC1> {
	typedef ushort type;
};

template<> struct CvMatTraits<CV_32SC1> {
	typedef int type;
};

template<> struct CvMatTraits<CV_32FC1> {
	typedef float type;
};

template<> struct CvMatTraits<CV_64FC1> {
	typedef double type;
};

//	Type traits -------------------------------------------------------------------------------------

//	Vector
template<typename T>
struct isVector{ static const int value = 0; };

template<typename T, typename A>
struct isVector<std::vector<T, A>>{ static const int value = 1; };

//	Mat
template<typename T>
struct isMat{ static const int value = 0; };

template<>
struct isMat<cv::Mat>{ static const int value = 1; };

//	Type names --------------------------------------------------------------------------------------

template<typename T> struct TypeStr {
	static std::string str() { throw std::logic_error("Unimplemented"); };
};

template<> struct TypeStr<void> {
	static std::string str() { return ""; }
};

template<> struct TypeStr<uchar> {
	static std::string str() { return "uchar"; }
};

template<> struct TypeStr<ushort> {
	static std::string str() { return "ushort"; }
};

template<> struct TypeStr<uint> {
	static std::string str() { return "uint"; }
};

template<> struct TypeStr<int> {
	static std::string str() { return "int"; }
};

template<> struct TypeStr<float> {
	static std::string str() { return "float"; }
};

template<> struct TypeStr<double> {
	static std::string str() { return "double"; }
};


#endif /* CVTYPETRAITS_H_ */
