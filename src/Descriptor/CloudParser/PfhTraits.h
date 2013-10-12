/*
 * PfhTraits.h
 *
 *   Created on: 11 paź 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#ifndef PFHTRAITS_H_
#define PFHTRAITS_H_

template<typename T>
struct PfhTraits {
	static int size() { return -1; };
	typedef int type;
};

template<> struct PfhTraits<pcl::FPFHSignature33>{ static int size() { return 33; }; };
template<> struct PfhTraits<pcl::PFHSignature125>{ static int size() { return 125; }; };
template<> struct PfhTraits<pcl::PFHRGBSignature250>{ static int size() { return 250; }; };

#endif /* PFHTRAITS_H_ */
