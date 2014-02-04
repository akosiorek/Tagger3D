/*
 * main.cpp
 *
 *   Created on: 29 sie 2013
 *       Author: Adam Kosiorek
 *	Description:
 */

#include "PcdCreator/TokyoCreator.h"

#include <iostream>

int main(int argc, char** argv) {

	if(argc < 3) {

		std::cerr << "ImgPrep <colorPath> <spacePath> <destPath>" << std::endl;
		exit(2);
	}

	TokyoCreator creator;
	std::string colorPath = argv[1];
	std::string spacePath = argv[2];
	std::string destPath = argv[3];
	creator.batchProcess(colorPath, spacePath, destPath);

	return 1;
}


