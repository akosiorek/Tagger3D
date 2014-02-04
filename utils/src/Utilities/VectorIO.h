/*
 * vectors.h
 *
 *  Created on: 6 Nov 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#ifndef VECTORS_H_
#define VECTORS_H_

#include "TypeTraits.h"
#include "logger.h"

#include <opencv2/core/core.hpp>

#include <stdexcept>
#include <vector>
#include <type_traits>
#include <fstream>
#include <iostream>

//	Default -------------------------------------------------------------------------------------------------------------------------------------
template<typename T, typename E = void> struct VectorIO {



	static void save(const std::vector<T> &vec, const std::string &filepath) {

		static_assert(std::is_fundamental<T>::value || std::is_same<std::string, T>::value, "Algorithm not defined for the type specified");
		std::ofstream os(filepath, std::ios::binary);
		if(!os.is_open() || !os.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}

		size_t elems = vec.size();
		os.write((char*)&elems, sizeof(size_t));
		os.write((char*)&vec[0], elems * sizeof(T));
		os.close();
	}

	static std::vector<T> load(const std::string &filepath) {

		static_assert(std::is_fundamental<T>::value || std::is_same<std::string, T>::value, "Algorithm not defined for the type specified");
		std::ifstream is(filepath, std::ios::binary);
		if(!is.is_open() || !is.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}
		size_t elems;
		is.read((char*)&elems, sizeof(size_t));
		std::vector<T> vec(elems);
		is.read((char*)&vec[0], elems * sizeof(T));
		return vec;
	}
};

//	Vector of Vectors -------------------------------------------------------------------------------------------------------------------------------------
template<typename T> struct VectorIO<std::vector<T>> {

	typedef uint size_type;
	static void save(const std::vector<std::vector<T>> &vec, const std::string &filepath) {

		static_assert(std::is_fundamental<T>::value
				|| std::is_same<std::string, T>::value, "Algorithm not defined for the type specified");

		std::ofstream os(filepath, std::ios::binary);
		if(!os.is_open() || !os.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}

		size_type elems = vec.size();
		std::vector<size_type> sizes;
		sizes.reserve(elems);
		for(const auto &subvec : vec)
			sizes.push_back(subvec.size());


		os.write((char*)&elems, sizeof(size_type));
		os.write((char*)&sizes[0], elems * sizeof(size_type));
//		std::for_each(std::begin(vec), std::end(vec),
//				[&](const T &subvec) {os.write((char*)&subvec[0], subvec.size() * sizeof(T));});
		for(const auto &subvec : vec)
			os.write((char*)&subvec[0], subvec.size() * sizeof(T));

		os.close();
	}

	static std::vector<std::vector<T>> load(const std::string &filepath) {

		static_assert(std::is_fundamental<T>::value
				|| std::is_same<std::string, T>::value, "Algorithm not defined for the type specified");

			std::ifstream is(filepath, std::ios::binary);
			if(!is.is_open() || !is.good()) {
				std::runtime_error e("Unable to open file: " + filepath);
				throw e;
			}

			std::vector<std::vector<T>> vec2;
			size_type elems;
			is.read((char*)&elems, sizeof(size_type));
			std::vector<size_type> sizes(elems);
			is.read((char*)&sizes[0], elems * sizeof(size_type));
			for(int i = 0; i < elems; i++) {

				std::vector<T> vec(sizes[i]);
				is.read((char*)&vec[0], sizes[i] * sizeof(T));
				vec2.push_back(vec);
			}

			return vec2;
	}
};

//	Vector of Mats -------------------------------------------------------------------------------------------------------------------------------------
template<typename E> struct VectorIO<cv::Mat, E> {

	typedef size_t size_type;

private:
	static void saveVectorMatBinaryStats(const std::vector<cv::Mat> &vec, const std::string &filepath) {

		std::ofstream os(filepath, std::ios::binary);
		if(!os.is_open() || !os.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}

		size_type mats = vec.size();

		std::vector<size_type> cols;
		std::vector<size_type> rows;
		cols.reserve(mats);
		rows.reserve(mats);
		for(const auto &mat : vec) {
			rows.push_back(mat.rows);
			cols.push_back(mat.cols);
		}

		// write number of images, dims and number of keypoints per image
		os.write((char*)&mats, sizeof(size_type));
		os.write((char*)&cols[0], mats * sizeof(size_type));
		os.write((char*)&rows[0], mats * sizeof(size_type));
		os.close();
	};

	static std::pair<std::vector<size_type>, std::vector<size_type>> loadVectorMatBinaryStats(const std::string &filepath) {

		std::ifstream is(filepath, std::ios::binary);
		if(!is.is_open() || !is.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}

		size_type mats = 0;
		is.read((char*)&mats, sizeof(size_type));
		std::vector<size_type> rows(mats);
		std::vector<size_type> cols(mats);
		is.read((char*)&cols[0], mats * sizeof(size_type));
		is.read((char*)&rows[0], mats * sizeof(size_type));

		for(int i = 0 ; i < mats; i++) {
			std::cout << "cols = " << cols[i] << " rows = " << rows[i] << std::endl;
		}

		is.close();

		return std::make_pair(rows, cols);
	}

public:

	static void save(const std::vector<cv::Mat> &vec, const std::string &filepath) {

		static_assert(std::is_arithmetic<E>::value, "Algorithm not defined for the type specified");
		if(vec.empty()) {
			std::runtime_error e("Empty vector");
			throw e;
		}

		std::ofstream os(filepath, std::ios::binary);
		if(!os.is_open() || !os.good()) {
			std::runtime_error e("Unable to open file: " + filepath);

			throw e;
		}
		int mats = vec.size();
		int cols = vec[0].cols;


		for(const auto& mat : vec)
			os.write((char*)mat.data, mat.rows * mat.cols * sizeof(E));
		os.close();

		saveVectorMatBinaryStats(vec, filepath + ".stat");
	}

	static std::vector<cv::Mat> load(const std::string &filepath) {

		static_assert(std::is_arithmetic<E>::value, "Algorithm not defined for the type specified");

		std::ifstream is(filepath, std::ios::binary);
		if(!is.is_open() || !is.good()) {
			std::runtime_error e("Unable to open file: " + filepath);
			throw e;
		}

		auto rows_cols = loadVectorMatBinaryStats(filepath + ".stat");
		auto &rows = rows_cols.first;
		auto &cols = rows_cols.second;
		size_type mats = rows.size();
		std::vector<cv::Mat> vec;
		vec.reserve(mats);

		for(int i = 0; i < mats; i++) {

			cv::Mat mat(rows[i], cols[i], CvTypeTraits<E>::type());
			is.read((char*)mat.data, rows[i] * cols[i] * sizeof(E));

			if(!mat.data) {
				std::runtime_error e("Corrupted image #: " + std::to_string(i));
				throw e;
			}

			vec.push_back(mat.clone());
		}
		return vec;
	}
};

#endif /* VECTORS_H_ */
