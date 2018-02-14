//
//
//
//
//

#include "navigation_node.h"

#include <iostream>

#include <map>
#include <vector>
#include <string>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

#include "ros/ros.h"


namespace cartographer_ros{
namespace cartographer_ros_navigation {
namespace{
    
using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

inline float Distance2BetweenPose(const geometry_msgs::Pose& pose1,
                                  const geometry_msgs::Pose& pose2){
    return (pose1.position.x-pose2.position.x)*(pose1.position.x-pose2.position.x)+
    (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y)
    /*+(pose1.position.z-pose2.position.z)*(pose1.position.z-pose2.position.z)*/;
}
    
inline float Distance2BetweenPoint(const geometry_msgs::Point& point1,
                                        const geometry_msgs::Point& point2){
    return (point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y);
}
    
inline float RotationBetweenPose(const geometry_msgs::Pose& pose1,
                                 const geometry_msgs::Pose& pose2){
    return abs(pose1.orientation.z-pose1.orientation.z);
}
    
geometry_msgs::Point operator+(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    geometry_msgs::Point sum;
    sum.x = a.x + b.x;
    sum.y = a.y + b.y;
    sum.z = a.z + b.z;
    return sum;
}
    
geometry_msgs::Point operator*(float a, const geometry_msgs::Point& b){
    geometry_msgs::Point product;
    product.x = a * b.x;
    product.y = a * b.y;
    product.z = a * b.z;
    return product;
}

/** 
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
  */  
const char kSubmapListTopicName [] = "/submap_list";
const char kSubmapQueryServiceName [] = "/submap_query";
const int kMaxNeighborNum = 3;
const int kFinishVersion = 180;
const int kProbabilityGridWidth = 100;
const int kProbabilityGridHeight = 100;
const int kOccupyThreshhold = 64;
const float kOccupyGridResolution = 0.05;
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

// Return whether a point is free in a submap
int NavigationNode::IsLocalFree(const geometry_msgs::Point& point,   // only use x & y in 2D case
                                 const SubmapIndex submap_index) {
    ::cartographer::common::MutexLocker locker(&mutex_);
    // If submap_grid exists, look up the grid
    if(submap_grid_.count(submap_index)==1){
        auto& submap_grid = submap_grid_[submap_index];
        int x = (point.x - submap_grid.x0) / submap_grid.resolution;
        int y = (point.y - submap_grid.y0) / submap_grid.resolution;
        if(x>=0&&x<submap_grid.width&&y>=0&&y<submap_grid.height){
            int val = submap_grid.data.at(y*submap_grid.width+x);
            return val;
        } else{
            std::cout<<"Point is out of submap range"<<std::endl;
            return -1;
        }
    }
    
    // Otherwise create the grid
    // first fetch the submaptexture
    cartographer_ros_msgs::SubmapEntry& submap_entry = submap_[submap_index];
    const SubmapId id{submap_entry.trajectory_id, submap_entry.submap_index};

    auto fetched_textures = ::cartographer_ros::FetchSubmapTextures(id, &submap_query_client_);
    const auto submap_texture = fetched_textures->textures.begin();
    
    // use fake map to do it (map only contain one element)
    std::map<SubmapId, SubmapSlice> fake_submap_slices;
    ::cartographer::io::SubmapSlice& submap_slice = fake_submap_slices[id];
    
    submap_slice.pose = ToRigid3d(submap_entry.pose);
    submap_slice.metadata_version = submap_entry.submap_version;
    
    // push fetched texture to slice and draw the texture
    submap_slice.version = fetched_textures->version;
    submap_slice.width = submap_texture->width;
    submap_slice.height = submap_texture->height;
    submap_slice.slice_pose = submap_texture->slice_pose;
    submap_slice.resolution = submap_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        submap_texture->pixels.intensity, submap_texture->pixels.alpha,
        submap_texture->width, submap_texture->height,
        &submap_slice.cairo_data);
   
    
    // Paint the texture
    auto painted_slices = cartographer::io::PaintSubmapSlices(fake_submap_slices, kOccupyGridResolution);
    
    // Convert painted surface into occupied grid
    auto& submap_grid = submap_grid_[submap_index];
    const Eigen::Array2f& origin = painted_slices.origin;
    cairo_surface_t* surface = painted_slices.surface.get();
    const int width = cairo_image_surface_get_width(surface);
    const int height = cairo_image_surface_get_height(surface);
    
    submap_grid.submap_index = submap_index;
    submap_grid.resolution = kOccupyGridResolution;
    submap_grid.width = width;
    submap_grid.height = height;
    submap_grid.x0 = - origin.x() * kOccupyGridResolution;
    submap_grid.y0 =  (origin.y() - height) * kOccupyGridResolution;
    
    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface));
    submap_grid.data.reserve(width*height);
    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            const uint32_t packed = pixel_data[(height-y-1)*width+x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
            observed == 0
            ? -1
            : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
            submap_grid.data.push_back(value);
        }
    }
    
    // Check the particular point
    const int x = (point.x - submap_grid.x0) / submap_grid.resolution;
    const int y = (point.y - submap_grid.y0) / submap_grid.resolution;
    if(x>=0&&x<submap_grid.width&&y>=0&&y<submap_grid.height){
        const int val = submap_grid.data.at(y*submap_grid.width+x);
        return val;
    } else{
        std::cout<<"Point is out of submap range"<<std::endl;
        return -1;;
    }    
}
    
    
// TODO: Return a free path from starting position to end postion using RRT
Path NavigationNode::PlanPathRRT(const geometry_msgs::Point& start,
                                 const geometry_msgs::Point& end) {
    return {start,end};
}

// Return a path bewteen origins of two connecting submaps
Path NavigationNode::PlanPathRRT(SubmapIndex start_idx,
                                 SubmapIndex end_idx){
    const auto start_point = submap_[start_idx].pose.position;
    const auto end_point = submap_[end_idx].pose.position;
    vector<SubmapIndex> submap_indexs = {start_idx,end_idx};
    // Naively check the straight line between two points
    if(IsPathLocalFree(start_point,end_point,submap_indexs)){
        return {start_point,end_point};
    }
    
}
    
//
    
bool NavigationNode::IsPathLocalFree(const geometry_msgs::Point& start,
                                     const geometry_msgs::Point& end,
                                     vector<SubmapIndex>& submap_indexs){
    float distance2 = Distance2BetweenPoint(start,end);
    float step = kOccupyGridResolution / distance2;
    for(float i=0;i<=1;i+=step){
        geometry_msgs::Point point = (1.0-i) * start + i * end;
        bool is_free = false;
        for(auto submap_index:submap_indexs){
            int val = IsLocalFree(point, submap_index);
            if(val>=kOccupyThreshhold) return false;
            if(val!=-1){
                is_free = true;
                break;
            }
        }
        if(!is_free) return false;
    }
    return true;
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
        if(distance2<kDistance2ThresholdForUpdating || 
            other_submap_entry.submap_index-submap_entry.submap_index==1 ||
            other_submap_entry.submap_index-submap_entry.submap_index==-1){
            // TODO: Try to connect these two submap
            // use RRT to connect these two submap
            Path path = PlanPathRRT(submap_entry.submap_index,other_submap_entry.submap_index);
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
    PrintState();
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
