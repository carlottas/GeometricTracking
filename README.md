###Geometric_tracker.cpp
**ROS parameters **
All the default values and the ros name  are defined in the cpp file itself.
Convention used :
Ros Name:  NAME_PARAM_Name
Default Value : DEFAULT_PARAM_Name
Variable name:  inputName
Example 
Name: ForgetThreshold 
Ros Name: NAME_PARAM_FORGET_THRESHOLD
Default Value: DEFAULT_PARAM_FORGET_THRESHOLD
Variable Name : inputForgetThreshold 
Name  |Launch Name |Default | Description|
---  | --- |--- | --- |
ForgetThreshold| "/pitt/srv/geometric_tracker/tracker_forget_threshold"|3|Time of not consecutive update after which the tracked cluster is removed|
RangeThreshold|"/pitt/srv/geometric_tracker/range_threshold"|0.20f|(meters) If a cluster is inside the sphere defined by this radium and a trucked cluster center the same ID is assigned to both of them.|
OldWeight|"/pitt/srv/geometric_tracker/confidence_weight_old"|0.6f|In order to compute the new cluster position a weighted mean is performed the previous and the current data|
NewWeight| "/pitt/srv/geometric_tracker/confidence_weight_new"|0.4f|In order to compute the new cluster position a weighted mean is performed the previous and the current data|



**Subscription **
Topic  |Callback |Msg | Description|
---  | --- |--- | --- |
TOPIC_OUT_NAME_OBJECT_PERCEPTION (defined in the "srv_manager.h")|clustersAcquisition|ClustersOutputConstPtr|Clusters pusblished by the object_segmentation node|

**Publication**
Topic  |Msg | Description|
---  | --- |--- | 
"geometric_tracker/trackedCluster"|ClustersOutput|It publishes the tracked clusters|
