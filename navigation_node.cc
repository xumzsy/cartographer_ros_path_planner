//
//
//
//
//

#include "navigation_node.h"

#include <iostream>
#include <fstream>

#include <map>
#include <vector>
#include <string>
#include <set>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"


namespace cartographer_ros{
namespace cartographer_ros_navigation {
namespace{
    
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
 
geometry_msgs::Pose PoseMultiply2D(geometry_msgs::Pose& pose1, geometry_msgs::Pose& pose2){
    // result = pose1 * pose2
    geometry_msgs::Pose result;
    result.orientation.w = pose1.orientation.w * pose2.orientation.w - pose1.orientation.z * pose2.orientation.z;
    result.orientation.x = 0.0;
    result.orientation.y = 0.0;
    result.orientation.z = pose1.orientation.z * pose2.orientation.w + pose1.orientation.w * pose2.orientation.z;
    
    float cos_theta_1 = 1 - 2*pose1.orientation.z;
    float sin_theta_1 = 2 * pose1.orientation.w * pose1.orientation.z;
    result.position.x = pose1.position.x + pose2.position.x * cos_theta_1 - pose2.position.y * sin_theta_1;
    result.position.y = pose1.position.y + pose2.position.y * cos_theta_1 + pose2.position.x * sin_theta_1;
    result.position.z = 0.0;
    return result;
}
    
const char kSubmapListTopicName [] = "/submap_list";
const char kSubmapQueryServiceName [] = "/submap_query";
const int kMaxNeighborNum = 3;
const int kFinishVersion = 180;
const int kProbabilityGridWidth = 100;
const int kProbabilityGridHeight = 100;
const int kOccupyThreshhold = 64;
const float kDistance2ThresholdForAdding = 5.0;
const float kDistance2ThresholdForUpdating = 1.0;
const float kRotationThresholdForUpdating = 1.0;
const float kProbabilityGridResolution = 0.05;
    
} // namespace
    
// constructor
NavigationNode::NavigationNode(){
    cartographer::common::MutexLocker lock(&mutex_);
    submap_list_subscriber_ = node_handle_.subscribe<cartographer_ros_msgs::SubmapList>(kSubmapListTopicName,10, &NavigationNode::UpdateRoadMap,this);
    submap_query_client_ = node_handle_.serviceClient<cartographer_ros_msgs::SubmapQuery>(kSubmapQueryServiceName);

    // For test
    clicked_point_subscriber_ = node_handle_.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,&NavigationNode::IsClickedPointFree, this);
}

// Return the cloest SubmapID if pose is free in this submap
SubmapIndex NavigationNode::CloestSubmap(const geometry_msgs::Pose& pose) const {
    float min_distance = FLT_MAX;
    SubmapIndex cloest_submap = -1;
    for(auto& submap:submap_){
        float distance2 = Distance2BetweenPose(submap.second.pose,pose);
        if(distance2<min_distance){
            min_distance = distance2;
            cloest_submap = submap.first;
        }
    }
    return cloest_submap;
}

// TODO: Return whether a point is free in local frame
// TOTEST
bool NavigationNode::IsLocalFree(const geometry_msgs::Point& point,   // only use x & y in 2D case
                 const SubmapIndex submap_index) {
    auto& submap_entry = submap_.find(submap_index)->second;

   
    ::cartographer_ros_msgs::SubmapQuery query;
    query.request.trajectory_id = submap_entry.trajectory_id;
    query.request.submap_index = submap_entry.submap_index;
    if(!submap_query_client_.call(query)) std::cout<<(query.response.status.message)<<std::endl;
    auto submap_texture = query.response.textures.begin();
    const int width = submap_texture->width;
    const int height = submap_texture->height;

    // unpack cells
    const std::string compressed_cells(submap_texture->cells.begin(),submap_texture->cells.end());
    auto pixels = ::cartographer::io::UnpackTextureData(compressed_cells,width,height);

    //TODO: submap_texture->slice_pose * submap_entry.pose;
    geometry_msgs::Pose submap_slice_pose = PoseMultiply2D(submap_entry.pose,submap_texture->slice_pose);
   

    // Future: using cartographer::transform
    // calculate pose in submap_id
    float relative_x = point.x - submap_slice_pose.position.x;
    float relative_y = point.y - submap_slice_pose.position.y;
    float cos_theta = 1 - 2*submap_slice_pose.orientation.z;
    float sin_theta = 2 * submap_slice_pose.orientation.w * submap_slice_pose.orientation.z;
    float local_x = relative_x * cos_theta + relative_y * sin_theta;
    float local_y = relative_y * cos_theta - relative_x * sin_theta;
    local_x *= -1;
    local_y *= -1;

    int x = local_x/submap_texture->resolution;
    int y = local_y/submap_texture->resolution;
    const int intensity = pixels.intensity.at(y*width+x);
    const int alpha = pixels.alpha.at(y*width+x);
    const int color = alpha*256+intensity;
    const int value = (1.0-color/255.0)*100;
    if(alpha==0 && intensity==0){
        std::cout<<"Not Observed"<<std::endl;
        return false;
    }
    std::cout<<local_x<<","<<local_y<<std::endl;
    std::cout<<x<<","<<y<<":"<<value<<std::endl;

    std::ofstream f ("example2.txt");
    {
        for(int j=height-1;j>=0;j--){
            for(int i=0;i<width;i++){
                //std::cout<<pixels.intensity[j*width+i] + 32*pixels.alpha[j*width+i]<<" ";
                f << pixels.intensity[j*width+i] + 32*pixels.alpha[j*width+i]<<" ";
            }
            //std::cout<<std::endl;
            f<<std::endl;
        }
    }
    f.close();
    return value < kOccupyThreshhold;
    
}
    
    
// TODO: Return a free path from starting position to end postion using RRT
Path NavigationNode::PlanPathRRT(const geometry_msgs::Pose& start,
                                const geometry_msgs::Pose& end) const {
    return {start,end};
}
    
// add new entry to road map
void NavigationNode::AddRoadMapEntry(const SubmapIndex submap_index){
    // check version
    auto& submap_entry = submap_[submap_index];
    if(submap_entry.submap_version!=kFinishVersion) return;
    
    // go throush existing
    for(auto& pair:submap_){
        // check validation of other_submap_entry
        auto& other_submap_entry = pair.second;
        if(other_submap_entry.submap_version!=kFinishVersion) continue;
        if(other_submap_entry.submap_index==submap_entry.submap_index) continue;
        
        float distance2 = Distance2BetweenPose(submap_entry.pose,other_submap_entry.pose);
        if(distance2<kDistance2ThresholdForUpdating){
            // TODO: Try to connect these two submap
            // use RRT to connect these two submap
            Path path = PlanPathRRT(submap_entry.pose,other_submap_entry.pose);
            if(path.empty()) continue;
            
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
    
// update the road mao every time received submaplist
void NavigationNode::UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg){
    ::cartographer::common::MutexLocker locker(&mutex_);
    
    // Keep track of submap IDs that don't appear in the message anymore.
    //std::set<SubmapIndex> submap_indexs_to_delete;
    //for (const auto& pair : submap_) {
    //    submap_indexs_to_delete.insert(pair.first);
    //}
    
    // check if submap has been changed
    for(auto& submap_entry:msg->submap){
        SubmapIndex submap_index = submap_entry.submap_index;
        //submap_indexs_to_delete.erase(submap_index);
        
        // add new finished submap into road_map_
        if(submap_.find(submap_index)==submap_.end()){
            submap_[submap_index] = submap_entry;
            AddRoadMapEntry(submap_index);
        } else{
            auto& old_submap_entry = submap_[submap_index];
            if(old_submap_entry.submap_version!=kFinishVersion) AddRoadMapEntry(submap_index);
            float distance2 = Distance2BetweenPose(old_submap_entry.pose,submap_entry.pose);
            float rotation = RotationBetweenPose(old_submap_entry.pose,submap_entry.pose);
            if(rotation>kRotationThresholdForUpdating || distance2>kDistance2ThresholdForUpdating){
                submap_[submap_index] = submap_entry;
                AddRoadMapEntry(submap_index);
            }
        }
    }
    
    // Delete all submaps that didn't appear in the message.
    //for (const auto& idx : submap_indexs_to_delete) {
    //    submap_.erase(idx);
    //}
    //PrintState();
}

/**
Functions below are for test
*/

void NavigationNode::IsClickedPointFree(const geometry_msgs::PointStamped::ConstPtr& msg){
    //std::cout<<msg->point.x<<","<<msg->point.y<<":";
    IsLocalFree(msg->point,0) ? std::cout<<"Free" : std::cout<<"Occupied";
    std::cout<<std::endl;
}

// print out the current state for testing and debugging
void NavigationNode::PrintState(){
    // print out submap_
    for(const auto& submap_entry:submap_){
        std::cout<<submap_entry.first<<": "
        <<submap_entry.second.pose.position.x<<","
        <<submap_entry.second.pose.position.y<<std::endl;
    }
    // print out road_map_
    for(auto pair:road_map_){
        std::cout<<pair.first<<": ";
        for(auto connection:pair.second){
            std::cout<<connection.first<<" ";
        }
        std::cout<<std::endl;
    }
}

} // namespace cartographer_ros_navigation
} // namespace cartographer_ros
