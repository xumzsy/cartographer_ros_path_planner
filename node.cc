

#include "node.hpp"

#include <unordered_map>
#include <vector>
#include <string>
#include <priority_queue>

#include "geometry_msgs/Pose.msg"

#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros_navigation {

inline float Distance2BetweenPose(const geometry_msgs::Pose& pose1,
                                  const geometry_msgs::Pose& pose2){
    return (pose1.position.x-pose2.position.x)*(pose1.position.x-pose2.position.x)+
    (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y)
    /*+(pose1.position.z-pose2.position.z)*(pose1.position.z-pose2.position.z)*/;
}
    
inline float RotationBetweenPose(const geometry_msgs::Pose& pose1,
                                      const geometry_msgs::Pose& pose2){
    return abs(pose1.orientation.z-pose1.orientation.z);
}
    
namespace{
const string SubmapListTopicName = "";
const int kMaxNeighborNum = 3;
const int kFinishVersion = 180;
const float kDistance2ThresholdForAdding = 1.0;
const float kDistance2ThresholdForUpdating = 1.0;
const float kRotationThresholdForUpdating = 1.0;
    
} // namespace
    
Node(){
    submap_list_subscriber = node_handle_.subscribe(SubmapListTopicName,10,
                                                    &UpdateRoadMap,this);
}

void AddRoadMapEntry(const SubmapId submap_id){
    // check version
    submap_entry = submap_[submap_id];
    if(submap_entry.submap_version!=kFinishVersion) return;
    
    // go throush existing
    for(auto other_submap_entry:submap_){
        // check validation of other_submap_entry
        if(other_submap_entry.submap_version!=kFinishVersion) continue;
        if(other_submap_entry.submap_index==submap_entry.submap_index) continue;
        
        float distance2 = Distance2BetweenPose(submap_entry.pose,other_submap_entry.pose);
        if(distance2<kDistance2Threshold){
            // TODO: Try to connect these two submap
            // use RRT to connect these two summap
            Path path = PlanPathRRT(submap_entry.pose,other_submap_entry.pose);
            if(Path.empty()) continue;
            
            // add into road_map
            SubmapConnectState submap_connect_state (submap_entry.submap_index,
                                                     other_submap_entry.submap_index,
                                                     distance2);
            SubmapConnectState other_submap_connect_state (other_submap_entry.submap_index,
                                                           submap_entry.submap_index,
                                                           distance2);
            
            submap_connect_state.path = path;
            std::reverse(path.begin(),path.end());
            other_submap_connect_state.path = std::move(path);
            road_map_[submap_entry.submap_index][other_submap_entry.submap_index] = std::move(submap_connect_state);
            road_map_[other_submap_entry.submap_index][submap_entry.submap_index] = std::move(other_submap_connect_state);
        }
    }
}
    
void UpdateRoadMap(const cartographer_ros_msgs::SubmapList& msg){
    // check if submap has been changed
    for(auto& submap_entry:msg->submap){
        SubmapId submap_id = submap_entry.submap_index;
        // add new finished submap into road_map_
        if(submap_.find(submap_id)==submap_.end()){
            submap_[submap_id] = submap_entry;
            AddRoadMapEntry(submap_id);
        } else{
            auto& old_submap_entry = submap_[submap_id];
            if(old_submap_entry.submap_version!=kFinishVersion) AddRoadMapEntry(submap_id);
            float distance2 = Distance2BetweenPose(old_submap_entry.pose,submap_entry.pose);
            float rotation = RotationBetweenPose(old_submap_entry.pose,submap_entry.pose);
            if(rotation>kRotationThresholdForUpdating || distance2>kDistance2ThresholdForUpdating){
                submap_[submap_id] = submap_entry;
                AddRoadMapEntry(submap_id);
            }
        }
    }
}


} // namespace cartographer_ros_navigation
