/*
 * clusterManager.h
 *
 *  Created on: Jun 9, 2015
 *      Author: luca-phd
 */
#ifndef GEOMETRIC_TRAKING_SRC_CLUSTERMANAGING_CLUSTERMANAGER_H_
#define GEOMETRIC_TRAKING_SRC_CLUSTERMANAGING_CLUSTERMANAGER_H_

#include "pitt_msgs/InliersCluster.h" // for input from topic node and cluster initialization
#include <pcl_ros/point_cloud.h>				// point cloud library
#include "../../../pitt_object_table_segmentation/src/point_cloud_library/pc_manager.h" // for my static library

using namespace pitt_msgs;
using namespace pcm;

typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtr;				// for point cloud smart pointer
typedef vector< InliersCluster> InliersClusters;
typedef boost::shared_ptr< float> DistancesPtr;			// for the list of the primitives

namespace cm {

class clusterManager {
	private:
		// data from cluster segmentation
	    InliersCluster messageInput;
		PCLCloudPtr cloud;
		vector< int> inliers;
		float stateX, stateY, stateZ;
		int R, G, B;
		long clusterId;
		// data to perform tracking
		float epsilon, oldWeigth, newWeigth;
		vector< float> distance; // same size as the incoming cluster vector. Contains the distances of all of them w.r.t. this tracked cluster
		// private methods
		int getRandomColorComponent();
		void initialize( InliersCluster clusterIn, long id, float epsilon, float oldWeigth, float newWeigth);

	public:
		clusterManager( InliersCluster clusterIn, long id, float epsilon, float oldWeigth, float newWeigth); // constructor
		virtual ~clusterManager(); // deconstructor

		InliersCluster getMessageInput();

		long getClusterId();

		float getStateX();
		float getStateY();
		float getStateZ();

		int getColorR();
		int getColorG();
		int getColorB();

		PCLCloudPtr getCloud();
		vector< int> getInliers();

		void updatePosition( InliersCluster &clusterTest);
		bool isWithinRange(  InliersCluster &clusterIn, float &d);
		float computeDistance( InliersCluster &cluster);

		void newTest(); // reset distances
		long addDistance( InliersCluster &cluster);
		long addDistance( float dist);
		vector< float> getDistances();
		bool getMinimumDistanceIdx( int &idx);


		static const float RANGE_THRESHOLD_DEFAULT;
		static const float OLD_WEIGHT_DEFAULT;
		static const float NEW_WEIGHT_DEFAULT;
	};

} /* namespace cm */

#endif /* GEOMETRIC_TRAKING_SRC_CLUSTERMANAGING_CLUSTERMANAGER_H_ */
