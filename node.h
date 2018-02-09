//
//  node.hpp
//  
//
//  Created by 徐梦泽 on 2018/2/8.
//

#ifndef node_hpp
#define node_hpp

#include <unordered_map>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Pose.msg"

#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros_navigation {

using SubmapId = long int;
using Path = std::vector<geometry_msgs::Pose>;
    
// Node to provide navigation for cartographer
class Node{
public:
    Node();
    ~Node();
    
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;
    
    // Return the cloest SubmapID if pose is free in this submap
    SubmapId CloestSubmap(geometry_msgs::Pose& pose) const;
    
    // Return whether a point is free in global frame
    bool IsFree(geometry_msgs::Pose& pose) const;
    
    // Return a free path from starting position to end postion using RRT
    Path PlanPathRRT(geometry_msgs::Pose& start,
                     geometry_msgs::Pose& end) const;
    
private:
    struct SubmapConnectState{
        SubmapId start_submap_id
        SubmapId end_submap_id;
        float distance;
        Path path;
        SubmapConnectState(SubmapId start_id, SubmapId end_id, float d) :
        start_submap_id(start_id),
        end_submap_id(end_id),
        distance(d) {};
    }
    
    // Add a submap into road map
    void AddRoadMapEntry(const SubmapId submap_id)
    
    // Update roadmap every time received submapList
    void UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
    
    ::ros::NodeHandle node_handle_;
    ::ros::Subscriber submap_list_subscriber_;
    
    // SubmapList
    std::unordered_map<SubmapId,cartographer_ros_msgs::SubmapEntry> submap_;
    
    // Road map to store the connectivity of submaps and corresponding path
    std::unordered_map<SubmapId,std::map<SubmapId,SubmapConnectState>> road_map_;
    
};
    
} // namespace cartographer_ros_navigation

#endif /* node_hpp */
