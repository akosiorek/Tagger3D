/*
 * FisherCluster.h
 *
 *  Created on: 21 gru 2013
 *      Author: adam
 */

#ifndef FISHERCLUSTER_H_
#define FISHERCLUSTER_H_

#include <Cluster.h>

extern "C" {
	#include <vl/generic.h>
	#include <vl/fisher.h>
	#include <vl/gmm.h>

}

namespace Tagger3D {

class FisherCluster: public Cluster {
public:
	FisherCluster() = delete;
	FisherCluster(const std::map<std::string, std::string> &_configMap);
	virtual ~FisherCluster();

	virtual cv::Mat cluster(const cv::Mat &descriptors) override;
	virtual void train(const std::vector<cv::Mat> &descriptors) override;
	void save() override;
	void load() override;
	bool isLoaded() override { return loaded; } ;

private:
	VlGMM* gmm = nullptr;
	long encodingSize;
};

} /* namespace Tagger3D */

#endif /* FISHERCLUSTER_H_ */
