/*
 * clusterManager.cpp
 *
 *  Created on: Jun 9, 2015
 *      Author: luca-phd
 */

#include "tracker_manager.h"

namespace cm {



	clusterManager::clusterManager( InliersCluster clusterIn, long id,
                                    float rangeThreshold, float oldConfidenceWeight, float newConfidenceWeight){
		initialize(clusterIn, id, rangeThreshold, oldConfidenceWeight, newConfidenceWeight);
	}
	void clusterManager::initialize( InliersCluster clusterIn, long id, float rangeTH, float old_weigth, float new_weigth){
		// store for eventualy late use
		messageInput = clusterIn;
		// set the cluster id for easy inspection
		clusterId = id;
		// get cluster cloud
		cloud = PCManager::cloudForRosMsg( clusterIn.cloud);
		inliers = clusterIn.inliers;
		// initialize the state
		stateX = clusterIn.x_centroid;
		stateY = clusterIn.y_centroid;
		stateZ = clusterIn.z_centroid;
		// initialize the color
		R = getRandomColorComponent();
		G = getRandomColorComponent();
		B = getRandomColorComponent();
		// initialize parameters for traking
		epsilon = rangeTH;
		oldWeigth = old_weigth;
		newWeigth = new_weigth;
	}

	clusterManager::~clusterManager() {
		// TODO Auto-generated destructor stub
	}

	InliersCluster clusterManager::getMessageInput(){
		return messageInput;
	}

	int clusterManager::getRandomColorComponent(){
		return rand() % 255 + 1;
	}

	void clusterManager::newTest(){
		distance.clear();
	}
	long clusterManager::addDistance( InliersCluster &cluster){
        long idx = distance.size();
		distance.push_back( computeDistance( cluster));
		return idx;
	}
	long clusterManager::addDistance( float dist){
		long idx = distance.size();
		distance.push_back( dist);
		return idx;
	}
	vector< float>clusterManager::getDistances(){
		return distance;
	}
	bool clusterManager::getMinimumDistanceIdx( int &idx){
		float minDist = 999999.0f;
		bool correct = false;
		for( int i = 0; i < distance.size(); i++){
			if( minDist > distance[ i]){
				minDist = distance[ i];
				idx = i;
				correct = true;
			}
		}
		return correct;
	}

	void clusterManager::updatePosition( InliersCluster &clusterTest){
		// compute an weigth mean to retrieve the new position of the cluster
		stateX = ( oldWeigth * stateX + newWeigth * clusterTest.x_centroid) / ( oldWeigth + newWeigth);
		stateY = ( oldWeigth * stateY + newWeigth * clusterTest.y_centroid) / ( oldWeigth + newWeigth);
		stateZ = ( oldWeigth * stateZ + newWeigth * clusterTest.z_centroid) / ( oldWeigth + newWeigth);
		// update cloud for visualization // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!?????????????????????????????????? update with only new points ??????????
		cloud = PCManager::cloudForRosMsg( clusterTest.cloud); // fai la media pesata di tuttti i punti ????????????????????????????????????????
		inliers = clusterTest.inliers;
	}

	bool clusterManager::isWithinRange(  InliersCluster &clusterTest, float &d){
		// compute if the new centroid is within a give range in the centroid stored on this class
		d = computeDistance( clusterTest);
		return (d <= epsilon * epsilon);

	}

	// compute the euclidean distances between the centroid of this and an incoming cluster
	// used also to check if it is within a spherical range ( dist < inputRangeThreshold^2)
	float clusterManager::computeDistance( InliersCluster &cluster){
		float x = ( cluster.x_centroid - stateX);
		float y = ( cluster.y_centroid - stateY);
		float z = ( cluster.z_centroid - stateZ);
		return x*x + y*y + z*z;
	}

	long clusterManager::getClusterId(){	return clusterId;	 }

	float clusterManager::getStateX(){ return stateX;  }
	float clusterManager::getStateY(){ return stateY;  }
	float clusterManager::getStateZ(){ return stateZ;  }

	int clusterManager::getColorR(){ return R;  }
	int clusterManager::getColorG(){ return G;  }
	int clusterManager::getColorB(){ return B;  }

	PCLCloudPtr clusterManager::getCloud(){ return cloud;  }
	vector< int> clusterManager::getInliers(){ return inliers;  }

} /* namespace cm */
