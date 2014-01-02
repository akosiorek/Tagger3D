/*
 * Utils.h
 *
 *  Created on: 21 Nov 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <vector>


namespace Tagger3D {

/*
 *
 */
class Utils {
public:
	Utils();
	virtual ~Utils();

	/**
	 * Utility method for vector's type conversion
	 * @param	class input_type	type of an input vector
	 * @param	class output_type	type of an output vector
	 * @param	vec	vector whose type is to be changed
	 * @return	vector beign a copy of the vec but of different type
	 */
	template <typename output_type, typename input_type>
	static std::vector<output_type> convertVec (std::vector<input_type> vec) {
		return std::vector<output_type>(vec.begin(), vec.end());
	}

	/**
	 * @brief	visualizeKeypoints in image
	 * @param	keys	keypoints to visualize
	 * @param	img	image that we works on
	 * @return	1 on success
	 */
	static int visualizeKeypoints(const cv::Mat &img, std::vector<cv::KeyPoint> &keys);

	/**
	 *	Converts int returned by cv::Mat.type() to a human readable std::string
	 *	@param	type	type returned by cv::Mat.type()
	 *	@return	human readable type string.
	 */
	static std::string cvType2Str(int type);

};

} /* namespace Tagger3D */
#endif /* UTILS_H_ */
