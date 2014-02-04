/*
 * FisherCluster.cpp
 *
 *  Created on: 21 gru 2013
 *      Author: adam
 */

#include <FisherCluster.h>

#include <cstdio>

namespace Tagger3D {

FisherCluster::FisherCluster(const std::map<std::string, std::string> &_configMap) : Cluster(_configMap) {

	encodingSize = 2 * dimCount + clusterCount;

}

FisherCluster::~FisherCluster() {

	vl_gmm_delete(gmm);
}

cv::Mat FisherCluster::cluster(const cv::Mat &descriptors) {

	float *enc;

	// allocate space for the encoding
	enc = (float*)vl_malloc(sizeof(float) * encodingSize);

	// run fisher encoding
	vl_fisher_encode
	 (enc, VL_TYPE_FLOAT,
	 vl_gmm_get_means(gmm),
	 dimCount, clusterCount,
	 vl_gmm_get_covariances(gmm),
	 vl_gmm_get_priors(gmm),
	 descriptors.data, 1,
	 VL_FISHER_FLAG_IMPROVED
	 ) ;

	cv::Mat clustered(1, encodingSize, CV_32FC1, enc);

	vl_free(enc);

	return clustered;
}

void FisherCluster::train(const std::vector<cv::Mat> &descriptors) {

	vl_size numData = 0;
	for(const auto &mat : descriptors)
		numData += mat.rows;

	float *data = new float[numData * dimCount];

	long counter = 0;
	for(const auto &mat : descriptors) {
		memcpy((void*)(data + counter), (void*)mat.data, dimCount * mat.rows);
		counter += mat.rows;
	}

	gmm = vl_gmm_new(VL_TYPE_FLOAT, dimCount, clusterCount) ;

	vl_gmm_cluster (gmm, data, numData);

	delete [] data;
}

void FisherCluster::save() {
}

void FisherCluster::load() {
}

} /* namespace Tagger3D */


//
//void Controller::trainGMMRun() {
//
//    std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(getParam<std::string>(trainDescriptorsDest));
//
//    int all_rows = 0;
//    cv::Mat sum = cv::Mat::zeros(1, descriptors[0].cols, CV_32F);
//    // compute number ofrows
//    // summing in each dim (for computing EX after)
//    for(auto &img: descriptors){
//      all_rows += img.rows;
//      for(int row = 0; row<img.rows; row++){
//          const float* row_ptr = img.ptr<float>(row);
//          for(int col = 0; col < img.cols; ++col ){
//              sum.at<float>(col) += row_ptr[col];
//          }
//      }
//    }
//    cv::Mat ex = sum/all_rows;
//
//    // compute std
//    sum = cv::Mat::zeros(1, descriptors[0].cols, CV_32F);
//    for(auto &img: descriptors){
//      for(int row = 0; row<img.rows; row++){
//          const float* row_ptr = img.ptr<float>(row);
//          for(int col = 0; col < img.cols; ++col ){
//              auto el = row_ptr[col] - ex.ptr<float>(0)[col];
//              el = el*el;
//              sum.at<float>(col) += el;
//          }
//      }
//    }
//    cv::Mat std = sum/all_rows;
//    sqrt(std, std);
//
//    /* I can not do:
//     * io->saveMatBinary<float>(ex, EX_file);
//     * io->saveMatBinary<float>(vx, VX_file);
//     *
//     * so I do io operations on vectors:
//     */
//
//    std::vector<float> vec_ex;
//    ex.row(0).copyTo(vec_ex);
//    std::vector<float> vec_std;
//    std.row(0).copyTo(vec_std);
//    io->saveVector<float>(vec_ex, getParam<std::string>(gmmEXPath));
//    io->saveVector<float>(vec_std,  getParam<std::string>(gmmSTDPath));
//
//
//
//    // descriptors normalization
//    for(auto &img: descriptors){
//        img = (img - repeat(ex,  img.rows, 1));
//        img = (img / repeat(std,  img.rows, 1));
//    }
//
//    // initialize gmm parameters and fit data
//    vl_size const numData = all_rows;
//    vl_size const dimension = descriptors[0].cols;
//    vl_size const numClusters = getParam<int>(gmmNumClusters);
//    float *data = new float[dimension * numData];
//    int counter =0;
//    for(auto &img: descriptors){
//        for(int row = 0; row<img.rows; row++){
//            const float* row_ptr = img.ptr<float>(row);
//            for(int col = 0; col < img.cols; ++col ){
//                data[counter] = row_ptr[col];
//                counter++;
//            }
//        }
//    }
//
//    // clustering&saving&cleaning
//    VlGMM* gmm = vl_gmm_new (VL_TYPE_FLOAT, dimension, numClusters) ;
//    vl_gmm_cluster (gmm, data, numData);
//    io->saveGMMText(*gmm,  getParam<std::string>(gmmPath), dimension, numClusters);
//    vl_gmm_delete(gmm);
//    delete [] data;
//}
//
//void Controller::fisherEncodeRun(){
//    //std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(getParam<std::string>(testDescriptorsDest));
//    int dimension;
//    int numClusters;
//    VlGMM* gmm = io->loadGMMText(getParam<std::string>(gmmPath), dimension, numClusters);
//    std::cout<<"gmm ready"<<std::endl;
//    float* covariances = (float*)vl_gmm_get_covariances (gmm);
//    std::cout<<"covariance[0]: "<<covariances[0] <<std::endl;
//    std::vector<float> vec_ex = io->loadVector<float>(getParam<std::string>(gmmEXPath));
//    std::vector<float> vec_vx = io->loadVector<float>(getParam<std::string>(gmmSTDPath));
//    //cv::Mat ex = io->loadMatBinary<float>(EX_file);
//    //cv::Mat vx = io->loadMatBinary<float>(VX_file);
//    cv::Mat ex = cv::Mat(vec_ex);
//    cv::Mat vx = cv::Mat(vec_vx);
//    cv::transpose(ex,ex);
//    cv::transpose(vx,vx);
//    std::cout<<"vmax ready"<<std::endl;
//    std::cout<<"ex: "<< ex.rows << " " << ex.cols<<std::endl;
//
//    vector<std::string> variants;
//    variants.push_back (getParam<std::string>(testDescriptorsDest));
//    variants.push_back (getParam<std::string>(trainDescriptorsDest));
//
//    bool train = true;
//    for(auto &variant: variants){
//        std::vector<cv::Mat> descriptors = io->loadVector<cv::Mat, float>(variant);
//        std::string encoded_file;
//        if(train) encoded_file = getParam<std::string>(gmmFisherVectorsTrain);
//        else encoded_file = getParam<std::string>(gmmFisherVectorsTest);
//
//        //dataToEncode
//        //std::ofstream myfile;
//        //myfile.open (encoded_file);
//        io->initTextFile(encoded_file);
//        for(auto &test_img: descriptors){
//            test_img = (test_img - repeat(ex,  test_img.rows, 1));
//            test_img = (test_img / repeat(vx,  test_img.rows, 1));
//            float *dataToEncode = new float[dimension * test_img.rows];
//            int counter = 0;
//            for(int row = 0; row<test_img.rows; row++){
//                const float* row_ptr = test_img.ptr<float>(row);
//                for(int col = 0; col < test_img.cols; ++col ){
//                    dataToEncode[counter] = row_ptr[col];
//                    counter++;
//                }
//            }
//            vl_size const numDataToEncode = test_img.rows ;
//            // allocate space for the encoding
//            float* enc;
//            enc = (float *)vl_malloc(sizeof(float) * 2 * dimension * numClusters);
//            // run fisher encoding
//            vl_fisher_encode
//                (enc, VL_TYPE_FLOAT,
//                vl_gmm_get_means(gmm), dimension, numClusters,
//                vl_gmm_get_covariances(gmm),
//                vl_gmm_get_priors(gmm),
//                dataToEncode, numDataToEncode,
//                VL_FISHER_FLAG_IMPROVED
//                ) ;
//
//            //for(int i =0; i<2 * dimension * numClusters; ++i){
//                //myfile<<enc[i]<<" ";
//            //}
//            //myfile<<"\n";
//            std::vector<float> v(enc, enc + 2 * dimension * numClusters);
//            io->appendToTextFile(v);
//
//            vl_free(enc);
//            delete[] dataToEncode;
//
//        }
//        //myfile.close();
//        io->finalizeTextFile();
//        train = false;
//    }
//    vl_gmm_delete(gmm);
//}
