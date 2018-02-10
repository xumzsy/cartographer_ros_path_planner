//
//  node.hpp
//  
//
//

#ifndef NAVIGATION_NODE_H
#define NAVIGATION_NODE_H

#include <map>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"

namespace cartographer_ros_navigation {

using SubmapIndex = int;
using Path = std::vector<geometry_msgs::Pose>;
    
// Node to provide navigation for cartographer
class NavigationNode{
public:
    NavigationNode();
    ~NavigationNode(){};
    
    NavigationNode(const NavigationNode&) = delete;
    NavigationNode& operator=(const NavigationNode&) = delete;
    
    // Return the cloest SubmapIndex if pose is free in this submap
    SubmapIndex CloestSubmap(const geometry_msgs::Pose& pose) const;
    
    // Return whether a point is free in local frame
    bool IsLocalFree(const geometry_msgs::Pose& pose,
                     const SubmapIndex submap_index) const;
    
    // Return a free path from starting position to end postion using RRT
    Path PlanPathRRT(geometry_msgs::Pose& start,
                     geometry_msgs::Pose& end) const;
    
    // print out the current state for testing and debugging
    void PrintState();
    
private:
    struct SubmapConnectState{
        SubmapIndex start_submap_index;
        SubmapIndex end_submap_index;
        float distance;
        Path path;
        SubmapConnectState(SubmapIndex start_index, SubmapIndex end_index, float d) :
        start_submap_index(start_index),
        end_submap_index(end_index),
        distance(d) {};
    };
    
    // Add a submap into road map
    void AddRoadMapEntry(const SubmapIndex submap_index);
    
    // Update roadmap every time received submapList
    void UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg){};
    
    ::ros::NodeHandle node_handle_;
    ::ros::Subscriber submap_list_subscriber_;
    ::ros::ServiceClient submap_query_client_;
    
    // SubmapList
    std::map<SubmapIndex,cartographer_ros_msgs::SubmapEntry> submap_;
    
    // Road map to store the connectivity of submaps and corresponding path
    std::map<SubmapIndex,std::map<SubmapIndex,SubmapConnectState>> road_map_;
    
};
    
} // namespace cartographer_ros_navigation

#endif /* node_hpp */
