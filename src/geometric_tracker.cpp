#include <string>
#include "pitt_msgs/ClustersOutput.h"
#include "../../pitt_object_table_segmentation/src/point_cloud_library/pc_manager.h" // for my static library
#include "../../pitt_object_table_segmentation/src/point_cloud_library/srv_manager.h" // for my static library
#include "tracker_library/tracker_manager.h"

using namespace ros;
using namespace pitt_msgs;
//using namespace geometric_tracking;
using namespace pcm;
using namespace srvm;
using namespace cm;

const int DEFAULT_PARAM_FORGET_THRESHOLD = 3; // time of not consecutive update (remove cluster from tracker)
const float DEFAULT_PARAM_RANGE_THRESHOLD = 0.20f; // Radios [m] of the sphere centered in a old object centroid. In which, if an object is in it, than it will have the same ID (recognition traking)
const float DEFAULT_PARAM_OLD_WEIGHT = 0.6f; // to compute the new position of the object (weighted mean) between the previous and the new data
const float DEFAULT_PARAM_NEW_WEIGHT = 0.4f; // to compute the new position of the object (weighted mean) between the previous and the new data

float inputRangeThreshold, inputOldWeight, inputNewWeight;

const string NAME_PARAM_FORGET_THRESHOLD = "/pitt/srv/geometric_tracker/tracker_forget_threshold";
const string NAME_PARAM_RANGE_THRESHOLD = "/pitt/srv/geometric_tracker/range_threshold";
const string NAME_PARAM_OLD_WEIGHT = "/pitt/srv/geometric_tracker/confidence_weight_old";
const string NAME_PARAM_NEW_WEIGHT = "/pitt/srv/geometric_tracker/confidence_weight_new";

typedef vector< InliersCluster> InliersClusters;
typedef boost::shared_ptr< ClustersOutput> ClustersOutputPtr;
static double inf = std::numeric_limits<double>::infinity();

ros::NodeHandle* nh_ptr = NULL;

Publisher trackedClusterPub; // variable to publish the output on the call back
boost::shared_ptr< visualization::PCLVisualizer> vis; // to visualize cloud
boost::thread vis_thread;
boost::mutex vis_mutex;
string log_str;
int num_objs = 0;

const bool DEFAULT_SHOW_TRACKER = false;
bool SHOW_TRACKER;			// open viewer with a color for every objects

static long clusterID; // can overflow ???????????????????????????????'

vector< clusterManager> tracker; // the memory of the tracked clusters
vector< clusterManager> shapeAdd, shapeRemove; // the added or removed shape in the tracker in this update
vector< int> updatedCnt; // contains the number of consecutive not updated scan for every clusters tracked in memory
vector< bool> updatedFlag; // for every cluster tracked on memory it contains true if it has been updated at this scan, false otherwise

void visSpin(){
    while(!vis->wasStopped()){
        boost::mutex::scoped_lock updateLock(vis_mutex);
        vis->spinOnce(100);
    }
}

void createNewTrackedCluster( InliersCluster &cl, vector< clusterManager> &tracker, vector< int> &updatedCnt, vector< bool> &updatedFlag){
    clusterManager tr( cl, clusterID++, inputRangeThreshold, inputOldWeight, inputNewWeight);
    tracker.push_back( tr);
    shapeAdd.push_back( tr);
    updatedCnt.push_back( 0);
    updatedFlag.push_back( false);
    if( SHOW_TRACKER) {
        string cluster_name = boost::str(boost::format("cluster_%i") %(int)tr.getClusterId());
        boost::mutex::scoped_lock updateLock(vis_mutex);
        if (vis->addText(cluster_name, 10, 20 + num_objs*20, 15,
                         tr.getColorR(), tr.getColorG(), tr.getColorB()))
            num_objs++;
    }
}

void initializeTracker( InliersClusters &clusters, vector< clusterManager> &tracker, vector< int> &updatedCnt, vector< bool> &updatedFlag){
    // initialize tracker the first time
    for( int i = 0; i < clusters.size(); i++){
        createNewTrackedCluster( clusters[ i], tracker, updatedCnt, updatedFlag);
        ROS_INFO( "create new tracked cluster from initialization %d", i);
    }
}

int getIndexOfMinimumElement(vector<float> d){
    float minD = inf;
    int idx = 0;
    for( int i = 0; i < d.size(); i++)
        if( d[ i] < minD){
            idx = i;
            minD = d[ i];
        }
    return idx;
}

bool containsOnlyInf( vector< float> d){
    for( int i = 0; i < d.size(); i++)
        if( d[ i] != inf)
            return false;
    return true;
}

// TRACKING CALLBACK
void clustersAcquisition(const ClustersOutputConstPtr& clusterObj){

    int inputForgetThreshold;

    // update ros parameters
    nh_ptr->param(NAME_PARAM_FORGET_THRESHOLD, inputForgetThreshold, DEFAULT_PARAM_FORGET_THRESHOLD);
    nh_ptr->param(NAME_PARAM_RANGE_THRESHOLD, inputRangeThreshold, DEFAULT_PARAM_RANGE_THRESHOLD);
    nh_ptr->param(NAME_PARAM_OLD_WEIGHT, inputOldWeight, DEFAULT_PARAM_OLD_WEIGHT);
    nh_ptr->param(NAME_PARAM_NEW_WEIGHT, inputNewWeight, DEFAULT_PARAM_NEW_WEIGHT);

    // get input
    InliersClusters clusters = clusterObj->cluster_objs;
    // reset tracker changes
    shapeAdd.clear();
    shapeRemove.clear();

    // track !!!
    if( tracker.size() == 0){
        initializeTracker( clusters, tracker, updatedCnt, updatedFlag);
    } else { // evolve tracker
        // reset updated flag array
        for( int i = 0; i < updatedFlag.size(); i++)
            updatedFlag[ i] = false;
        // scan for every incoming clusters
        for( int i = 0; i < clusters.size(); i++){
            // create a variable to contain the distances between an input cluster and all the clusters tracked in memory
            vector< float> distances;
            // scan for every tracked cluster in memory
            string log = "distances: ";
            for( int j = 0; j < tracker.size(); j++){
                float d; // will contain the distance between Tj and Ci
                if( tracker[ j].isWithinRange( clusters[ i], d)){ // spherical range of radius inputRangeThreshold
                    distances.push_back( d);
                    log += boost::to_string( d) + " ";
                } else {
                    distances.push_back( inf);
                    log += "inf ";
                }
            }
            ROS_INFO( "%s", log.c_str());
            // analyze the distance vector to retrieve the right action for tracker memory management
            if( containsOnlyInf( distances)){
                // the cluster is far away from any tracked cluster in memory. So create it
                createNewTrackedCluster( clusters[ i], tracker, updatedCnt, updatedFlag);  // does not update distances vector !!!!!
                ROS_INFO( " created new cluster tracked in memory !!!");
            } else {
                // get the index with the minimum distance
                int j_star = getIndexOfMinimumElement(distances);
                if( updatedFlag[ j_star]){
                    ROS_ERROR( " impossible duplicate references between a tracked cluster and two input blobs");
                } else {
                    // update tracked position (weighted mean)
                    tracker[ j_star].updatePosition( clusters[ i]);
                    // reset flag and count
                    updatedFlag[ j_star] = true;
                    updatedCnt[ j_star] = 0;
                    ROS_INFO( " updated cluster on memory %d with incoming blob %d", j_star, i);
                }
            }
        }
        // control the count to eliminate tracked cluster that are not updated for a while

        for( int k = 0; k < updatedFlag.size(); k++)
            if( ! updatedFlag[ k]){
                updatedCnt[ k] += 1;
                if(updatedCnt[ k] >= inputForgetThreshold){
                    // set in the eliminate list
                    InliersCluster clusterIn = tracker[ k].getMessageInput();
                    clusterManager removed( clusterIn, tracker[ k].getClusterId(), inputRangeThreshold, inputOldWeight, inputNewWeight);
                    shapeRemove.push_back( removed);
                    // eliminate cluster tracked in memory
                    tracker.erase( tracker.begin() + k);
                    updatedFlag.erase( updatedFlag.begin() + k);
                    updatedCnt.erase( updatedCnt.begin() + k);
                    ROS_INFO( " eliminate cluster %d from memory", k);
                }
            }

        // only for logging behaviour
        string log1 = " CNT  cluster update: (" + boost::to_string( updatedCnt.size()) + "/" + boost::to_string( tracker.size()) + ") ";
        for( int i = 0; i < updatedCnt.size(); i++)
            log1 += boost::to_string( updatedCnt[ i]) + " ";
        ROS_INFO( "%s", log1.c_str());
        string log2 = " FLAG cluster update: (" + boost::to_string( updatedFlag.size()) + "/" + boost::to_string( tracker.size()) + ") ";
        for( int i = 0; i < updatedFlag.size(); i++)
            log2 += boost::to_string( updatedFlag[ i]) + " ";
        ROS_INFO( "%s", log2.c_str());

    }

    if( SHOW_TRACKER){
        boost::mutex::scoped_lock updateLock(vis_mutex);
        for( int i = 0; i < tracker.size(); i++){
            string clusterName = boost::str(boost::format("cluster_%i") %(int)tracker[i].getClusterId());
            PCManager::updateVisor( vis, tracker[i].getCloud(), tracker[i].getColorR(), tracker[i].getColorG(), tracker[i].getColorB(), clusterName);
            // use updateText if you plan to change the global parameters to online modifiable ros parameters
            log_str = str(boost::format("TRACKER_FORGET_THRESHOLD: %s    RANGE_THRESHOLD: %s    "
                                                "OLD_WEIGHT: %s    NEW_WEIGHT: %s")
                          %inputForgetThreshold %inputRangeThreshold %inputOldWeight %inputNewWeight);
            vis->updateText(log_str, 10, 520, "log_str");
        }
    }

    // prepare node output to be published
    // write actual tracked class
    ClustersOutputPtr trackedClustersObject ( new ClustersOutput);
    if( tracker.size() > 0){ // at least one cluster
        for( int j = 0; j < tracker.size(); j++){ // for all the clusters
            InliersCluster outCl;
            outCl.cloud = PCManager::cloudToRosMsg( tracker[ j].getCloud());
            outCl.inliers = tracker[ j].getInliers();
            outCl.x_centroid = tracker[ j].getStateX();
            outCl.y_centroid = tracker[ j].getStateY();
            outCl.z_centroid = tracker[ j].getStateZ();
            outCl.shape_id = (int)tracker[ j].getClusterId();

            // append this cluster to output
            trackedClustersObject->cluster_objs.push_back( outCl);
        }
    }
    if( shapeAdd.size() > 0){
        // write which shapes are new
        for( int j = 0; j < shapeAdd.size(); j++){
            InliersCluster outCl;
            outCl.cloud = PCManager::cloudToRosMsg( shapeAdd[ j].getCloud());
            outCl.inliers = shapeAdd[ j].getInliers();
            outCl.x_centroid = shapeAdd[ j].getStateX();
            outCl.y_centroid = shapeAdd[ j].getStateY();
            outCl.z_centroid = shapeAdd[ j].getStateZ();
            outCl.shape_id = (int)shapeAdd[ j].getClusterId();

            trackedClustersObject->cluster_added.push_back( outCl);
        }
    }
    if( shapeRemove.size() > 0){
        // write which shapes are new
        for( int j = 0; j < shapeRemove.size(); j++){
            InliersCluster outCl;
            outCl.cloud = PCManager::cloudToRosMsg( shapeRemove[ j].getCloud());
            outCl.inliers = shapeRemove[ j].getInliers();
            outCl.x_centroid = shapeRemove[ j].getStateX();
            outCl.y_centroid = shapeRemove[ j].getStateY();
            outCl.z_centroid = shapeRemove[ j].getStateZ();
            outCl.shape_id = (int)shapeRemove[ j].getClusterId();

            // write which shapes have been deleted
            trackedClustersObject->cluster_removed.push_back( outCl);
        }
    }

    // publish it !!!
    trackedClusterPub.publish( trackedClustersObject);


    ROS_INFO("-----------------------------------------------");
}

// main method of the node
int main(int argc, char **argv){
    init(argc, argv, "geometric_tracker");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    clusterID = 0;
    SHOW_TRACKER = srvm::getBoolPtrParameter(argv[1], DEFAULT_SHOW_TRACKER);

    // set subscriber
    Subscriber sub = nh.subscribe ( srvm::TOPIC_OUT_NAME_OBJECT_PERCEPTION, 10, clustersAcquisition); // to the gazebo turtle kinect or real kinect

    // create window to visualize cloud
    if( SHOW_TRACKER) {

        log_str = str(boost::format("TRACKER_FORGET_THRESHOLD: %s    RANGE_THRESHOLD: %s    "
                                            "OLD_WEIGHT: %s    NEW_WEIGHT: %s")
                      %DEFAULT_PARAM_FORGET_THRESHOLD
                      %DEFAULT_PARAM_RANGE_THRESHOLD
                      %DEFAULT_PARAM_OLD_WEIGHT
                      %DEFAULT_PARAM_NEW_WEIGHT);

        vis = PCManager::createVisor("Geometric Table Tracking");
        vis->setCameraPosition(8-2.19051, 0.198678, 0.366248, -0.044886, 0.0859204, 0.471681, -0.0487582, 0.00610776, 0.998792);
        vis->setCameraFieldOfView(0.8575);
        vis->setCameraClipDistances(0.0064556, 6.4556);
        vis->setPosition(900, 480);
        vis->setSize(960, 540);
        vis->addText(log_str, 10, 520, 13, 0.9, 0.9, 0.9, "log_str");
        vis_thread = boost::thread(visSpin);
    }

    // set publisher
    trackedClusterPub = nh.advertise< ClustersOutput>( "geometric_tracker/trackedCluster", 10); // to another

    //ros::Rate r(20);
    while ( nh.ok()){
        spinOnce();
        //r.sleep();
    }
    if (SHOW_TRACKER){
        vis->close();
        vis_thread.join();
    }
    return 0;
}
