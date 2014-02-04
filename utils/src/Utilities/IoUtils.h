/*
 * IoUtils.h
 *
 *  Created on: 4 Oct 2013
 *      Author: Adam Kosiorek
 * Description: 
 */

#ifndef IOUTILS_H_
#define IOUTILS_H_

#include "logger.h"
#include "TypeTraits.h"
#include "VectorIO.h"

#include <opencv2/core/core.hpp>
#include <fstream>
#include <stdexcept>
#include <memory>
#include<iterator>
 #include <iostream>

extern "C" {
    #include <vl/generic.h>
    #include <vl/fisher.h>
    #include <vl/gmm.h>
}

namespace Tagger3D {

/*
 *
 */
class IoUtils {

//	Methods ----------------------------------------------------------------------------------
public:
	virtual ~IoUtils();


    VlGMM* loadGMMText(const std::string &filename, int& dimension, int& numClusters) const{
        FILE * pFile;
        pFile = fopen (filename.c_str(),"r");
        fscanf (pFile, "dimension: %d numClusters: %d", &dimension, &numClusters);
        int& DIM = dimension;
        int& NUMCLUST = numClusters;
        VlGMM* gmm = vl_gmm_new (VL_TYPE_FLOAT, DIM, NUMCLUST) ;

        //loading means
        float* means = new float[DIM*NUMCLUST];
        for(int clust = 0; clust < NUMCLUST; ++clust){
            for(int dim = 0; dim < DIM; ++dim){
                fscanf(pFile, "%f", &means[clust*DIM+dim]);
            }
        }
        vl_gmm_set_means (gmm, means);
        delete [] means;

        //loading priors
        float* priors = new float[DIM*NUMCLUST];
        for(int clust = 0; clust < NUMCLUST; ++clust){
            for(int dim = 0; dim < DIM; ++dim){
                fscanf(pFile, "%f", &priors[clust*DIM+dim]);
            }
        }
        vl_gmm_set_priors (gmm, priors);
        delete [] priors;

        //loading covariances
        float* covariances = new float[DIM*NUMCLUST];
        for(int clust = 0; clust < NUMCLUST; ++clust){
            for(int dim = 0; dim < DIM; ++dim){
                fscanf(pFile, "%f", &covariances[clust*DIM+dim]);
            }
        }
        vl_gmm_set_covariances(gmm, covariances);
        delete [] covariances;

        fclose (pFile);
        return gmm;
    }


    int saveGMMText(const VlGMM& gmm, const std::string &filename, int dimension, int numClusters) const{
        std::fstream fs;
        //std::cout<<"filename: "<<filename<<std::endl;
        fs.open (filename, std::fstream::out);
        if (fs.is_open()){
            fs << "dimension: "<<dimension<< " numClusters: "<<numClusters<<"\n";
            float* means = (float*)vl_gmm_get_means (&gmm);
            for(int i = 0; i< numClusters; ++i){
                for(int j =0; j<dimension;++j){
                    fs << means[i*dimension+j]<< " ";
                }
                fs<<"\n";
            }
            float* priors = (float*)vl_gmm_get_priors (&gmm);
            for(int i = 0; i< numClusters; ++i){
                for(int j =0; j<dimension;++j){
                    fs << priors[i*dimension+j]<< " ";
                }
                fs<<"\n";
            }
            float* covariances = (float*)vl_gmm_get_covariances (&gmm);
            for(int i = 0; i< numClusters; ++i){
                for(int j =0; j<dimension;++j){
                    fs << covariances[i*dimension+j]<< " ";
                }
                fs<<"\n";
            }
            fs.close();
        }
        else{
            //std::cout << "Error opening file";
        }

        return 1;

    }




	template<typename T>
	void saveCv(const T &obj, const std::string &filename) const;

	template<typename T>
	T loadCv(const std::string &filename) const;

	template<typename T>
	void saveMatBinary(const cv::Mat &mat, const std::string &filename, bool saveStats = false) const;

	template<typename T>
	cv::Mat loadMatBinary(const std::string &filename) const;

	template<typename T>
	cv::Mat loadMatBinary(const std::string &filename, int rows, int cols) const;

	template<typename T, typename E = void>
	void saveVector(const std::vector<T> &vec, const std::string& filename) const;

	template<typename T, typename E = void>
	std::vector<T> loadVector(const std::string& filename) const;

	void initTextFile(const std::string &filename);
	void finalizeTextFile();

	template<typename T>
	void appendToTextFile(const std::vector<T> &vec);

	template<typename T>
	void appendToTextFile(const T &el);

    template <typename T>
    std::vector<std::vector<T>> loadTextFileAsMat(const std::string& filename);


	const std::string& getPath() const {
		return path;
	}

	void setPath(const std::string& path) {
		this->path = path;
	}

	static std::shared_ptr<IoUtils> getInstance();

private:
	IoUtils();
	IoUtils(const IoUtils &);
	std::string makePath(const std::string &path, const std::string &filename) const;

	void saveVectorMatBinaryStats(const std::vector<cv::Mat> &vec, const std::string &filename) const;
	std::vector<int> loadVectorMatBinaryStats(const std::string &filename) const;

	template<typename T>
	std::string appendDataType(const std::string &filename) const;

//	Fields	----------------------------------------------------------------------------------
public:
private:

	std::string path;
	const std::string key = "key";
	const std::string stat = ".stat";
	const std::string cvExtension = ".yml";

	const std::string loggerName = "Main.IoUtils";
	const lgr::LoggerPtr logger;

	std::ofstream outStream;
	const std::string sep = " ";
	const std::string newline = "\n";
};

template<typename T>
inline void IoUtils::saveCv(const T& obj, const std::string& filename) const {
	TRACE(logger, "Saving an object under \"" << filename << "\"");

	std::string filepath = makePath(path, filename + cvExtension);
	cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
	if( !fs.isOpened()) {
		std::runtime_error e("Cannot initialise a cv::FileStorage at " + filepath);
		ERROR(logger, "saveCV: " << e.what());
		throw e;
	}

	fs << key << obj;
	fs.release();
}

template<typename T>
inline T IoUtils::loadCv(const std::string& filename) const {
	TRACE(logger, "Loading an object from \"" << filename << "\"");

	std::string filepath = makePath(path, filename + cvExtension);
	cv::FileStorage fs(filepath, cv::FileStorage::READ);
	if( !fs.isOpened()) {
		if( !fs.open(filename, cv::FileStorage::READ)) {
			std::runtime_error e("Could not open the following file: " + filepath);
			ERROR(logger, ": " << e.what());
			throw e;
		}
	}

	T obj;
	try {
		fs[this->key] >> obj;
	} catch(...) {

		std::runtime_error e("Cannot read the filestorage: " + path);
	}
	return obj;
}

template<typename T>
inline void IoUtils::saveMatBinary(const cv::Mat& mat,	const std::string& filename, bool saveStats) const {
	TRACE(logger, "Saving a cv::Mat in a binary file \"" << filename << "\"")

	if(!mat.data)
		std::runtime_error e("Corrupted image");

	std::string filepath = makePath(path, appendDataType<T>(filename));
	std::ofstream os(filepath, std::ios::binary);
	if(!os.is_open() || !os.good()) {
		std::runtime_error e("Unable to open file: " + filepath);
		ERROR(logger, "saveMatBinary: " << e.what());
		throw e;
	}

	int rows = mat.rows;
	int cols = mat.cols;

	// write rows and cols
	if(saveStats) {
		os.write((char*)&rows, sizeof(int));
		os.write((char*)&cols, sizeof(int));
	}

	for(int i = 0; i < rows; ++i)
		os.write((char*)mat.ptr<T>(i), cols * sizeof(T));

	os.close();
}

template<typename T>
inline cv::Mat IoUtils::loadMatBinary(const std::string& filename) const {
	TRACE(logger, "Loading a cv::Mat from a binary file \"" << filename << "\"");

	std::string filepath = makePath(path, appendDataType<T>(filename));
	std::ifstream is(filepath, std::ios::binary);
	if(!is.is_open() || !is.good()) {
		std::runtime_error e("Unable to open file: " + filepath);
		ERROR(logger, "loadMatBinary: " << e.what());
		throw e;
	}

	int rows, cols;

	// read rows and cols data
	is.read((char*)&rows, sizeof(int));
	is.read((char*)&cols, sizeof(int));
	cv::Mat mat(rows, cols, CvTypeTraits<T>::type());

	for(int i = 0; i < rows; ++i)
		is.read((char*)mat.ptr<T>(i), cols * sizeof(T));

	is.close();

	if(!mat.data)
		std::runtime_error e("Corrupted image");

	return mat;
}

template<typename T>
inline cv::Mat IoUtils::loadMatBinary(const std::string& filename, int rows, int cols) const {
	TRACE(logger, "Loading a cv::Mat from a binary file \"" << filename << "\"");

	std::string filepath = makePath(path, appendDataType<T>(filename));
	std::ifstream is(filepath, std::ios::binary);
	if(!is.is_open() || !is.good()) {
		std::runtime_error e("Unable to open file: " + filepath);
		ERROR(logger, "loadMatBinary: " << e.what());
		throw e;
	}
	cv::Mat mat(rows, cols, CvTypeTraits<T>::type());

	for(int i = 0; i < rows; ++i)
		is.read((char*)mat.ptr<T>(i), cols * sizeof(T));

	is.close();

	if(!mat.data)
		std::runtime_error e("Corrupted image");

	return mat;
}

template<typename T>
inline void IoUtils::appendToTextFile(const std::vector<T>& vec) {

	if(!outStream.is_open() || !outStream.good()) {
		std::runtime_error e("File has not been initialized");
		ERROR(logger, "appendToTextFile: " << e.what());
		throw e;
	}
	std::copy(vec.begin(), vec.end(), std::ostream_iterator<T>(outStream, sep.c_str()));
	outStream << newline;
}

template<typename T>
inline void IoUtils::appendToTextFile(const T& el) {

	if(!outStream.is_open() || !outStream.good()) {
		std::runtime_error e("File has not been initialized");
		ERROR(logger, "appendToTextFile: " << e.what());
		throw e;
	}
	outStream << el << sep.c_str();
}

template<typename T>
inline std::vector<std::vector<T>> IoUtils::loadTextFileAsMat(const std::string& filename) {
    std::vector<std::vector<float>> result_mat;
    std::ifstream source;                    // build a read-Stream
    source.open(filename, std::ios_base::in);  // open data
    if (!source)  {                     // if it does not work
        std::cerr << "Can't open Data!\n";
    }
    for(std::string line; std::getline(source, line); ){   //read stream line by line
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        std::istringstream reader(line);      //make a stream for the line itself
        std::vector<T> values_vec;
        do
        {
            // read as many numbers as possible.
            int i = 0;
            for (T value; reader >> value;) {
                //std::cout<< "value "<<i<<" " << value<<std::endl;
                values_vec.push_back(value);
                i++;
            }
            result_mat.push_back(values_vec);
            // consume and discard token from stream.
            if (reader.fail())
            {
                reader.clear();
                std::string token;
                reader >> token;
            }
       }
       while (!reader.eof());
    }
    return result_mat;
}


template<typename T>
std::string IoUtils::appendDataType(const std::string &filename) const {

	return filename + "_" + TypeStr<T>::str();
}

template<typename T, typename E>
void IoUtils::saveVector(const std::vector<T> &vec, const std::string& filename) const {

	std::string filepath = makePath(path, appendDataType<E>(filename));
	VectorIO<T, E>::save(vec, filepath);
}

template<typename T, typename E>
std::vector<T> IoUtils::loadVector(const std::string& filename) const {

	std::string filepath = makePath(path, appendDataType<E>(filename));
	return VectorIO<T, E>::load(filepath);
}


} /* namespace Tagger3D */
#endif /* IOUTILS_H_ */
