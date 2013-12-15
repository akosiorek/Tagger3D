/*
 * Utils.h
 *
 *  Created on: 14 gru 2013
 *      Author: adam
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <memory>

namespace Tagger3D {
class Utils {
public:
	Utils();
	virtual ~Utils();

	template<typename T>
	static T makeUnique(T obj) {return std::unique_ptr<T>(obj);};
};

} //namespace Tagger3D

#endif /* UTILS_H_ */
