//
//  node.hpp
//  
//
//

#ifndef NAVIGATION_NODE_H
#define NAVIGATION_NODE_H

#include <map>
#include <vector>


#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/RoadmapQuery.h"
#include "cartographer_ros_msgs/ConnectionQuery.h"
#include "cartographer_ros_msgs/PathPlan.h"
#include "cartographer_ros_msgs/ReconnectSubmaps.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"

namespace cartographer_ros{
namespace cartographer_ros_navigation {

using SubmapIndex = int;
using Path = std::vector<geometry_msgs::Point>;
    
struct RRTreeNode{
    geometry_msgs::Point point;
    RRTreeNode* parent_node;
    std::vector<RRTreeNode*> children_node;
    RRTreeNode(geometry_msgs::Point p){point=p;parent_node=nullptr;children_node.clear();}
};

// Node to provide navigation for cartographer
class NavigationNode{
public:
    
    NavigationNode();
    ~NavigationNode(){};
    
    NavigationNode(const NavigationNode&) = delete;
    NavigationNode& operator=(const NavigationNode&) = delete;
    
    // Return whether a point is free in local frame (-1: Unobserved, 0: Free, 100: Occupied)
    int IsLocalFree(const geometry_msgs::Point& point, SubmapIndex submap_index) const;
    bool IsPathLocalFree(const geometry_msgs::Point& start,
                         const geometry_msgs::Point& end,
                         const std::vector<SubmapIndex>& submap_indexs) const;
    
    // Return the cloest SubmapIndex if pose is free in this submap
    SubmapIndex CloestSubmap(const geometry_msgs::Point& point);
    
    // Return a free path from starting position to end postion using RRT
    Path PlanPathRRT(const geometry_msgs::Point& start,
                     const geometry_msgs::Point& end);
    
    // Return a free path between submaps' origin
    Path PlanPathRRT(SubmapIndex start_idx, SubmapIndex end_idx);
    
    // Returan a path connecting two remote submaps
    Path ConnectingSubmap(SubmapIndex start_idx, SubmapIndex end_idx);
    
    // connecting two points in given submaps
    Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                          const geometry_msgs::Point& end_point,
                          const std::vector<SubmapIndex> submap_indexs);
    
    // Functions for RRT
    geometry_msgs::Point RandomFreePoint(const std::vector<SubmapIndex>& submap_indexes);
    
    // Search the nearest node in RRT
    RRTreeNode* NearestRRTreeNode(RRTreeNode* root, const geometry_msgs::Point& target);
    
    // Destroy all nodes in RRT
    void DestroyRRTree(RRTreeNode* root);
    
    // print out the current state for testing and debugging
    void PrintState();
    void AddDisplayPath(Path path);
    
private:
    struct SubmapConnectState{
        SubmapIndex start_submap_index;
        SubmapIndex end_submap_index;
        float length;
        Path path;
        
        SubmapConnectState(){};
        SubmapConnectState(SubmapIndex start_index, SubmapIndex end_index, float d) :
        start_submap_index(start_index),
        end_submap_index(end_index),
        length(d) {};
    };
    
    struct SubmapGrid{
        SubmapIndex submap_index;
        double resolution;
        double x0;
        double y0;
        int width;
        int height;
        std::vector<int> data;
    };
    
    // Add a submap grid into submap_grid_
    void AddSubmapGrid(SubmapIndex submap_index);
    
    // Reconnect two submaps
    bool ReconnectSubmaps(SubmapIndex start_idx, SubmapIndex end_idx);
    
    // Add a submap into road map
    void AddRoadMapEntry(const SubmapIndex submap_index);
    
    // Update submap_, road_map_ and submap_grid_ every time receive submapList
    void UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
    
    ::cartographer::common::Mutex mutex_;
    ::ros::NodeHandle node_handle_ GUARDED_BY(mutex_); 
    ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
    ::ros::ServiceClient submap_query_client_ GUARDED_BY(mutex_);
    
    // SubmapList
    std::map<SubmapIndex,cartographer_ros_msgs::SubmapEntry> submap_ GUARDED_BY(mutex_);
    
    // SubmapGrid
    std::map<SubmapIndex, SubmapGrid> submap_grid_ GUARDED_BY(mutex_);
    
    // Road map to store the connectivity of submaps and corresponding path
    std::map<SubmapIndex,std::map<SubmapIndex,SubmapConnectState>> road_map_ GUARDED_BY(mutex_);
    
    // ROS agent
    nav_msgs::Path path_to_display_;
    ::ros::WallTimer path_publisher_timer_;
    ::ros::Publisher path_publisher_;
    ::ros::ServiceServer roadmap_query_server_;
    ::ros::ServiceServer connection_query_server_;
    ::ros::ServiceServer plan_path_server_;
    ::ros::ServiceServer reconnect_submaps_server_;
    bool QueryRoadmap(cartographer_ros_msgs::RoadmapQuery::Request &req,
                      cartographer_ros_msgs::RoadmapQuery::Response &res) const;
    bool QueryConnection(cartographer_ros_msgs::ConnectionQuery::Request &req,
                         cartographer_ros_msgs::ConnectionQuery::Response &res) const;
    bool PlanPath(cartographer_ros_msgs::PathPlan::Request &req,
                  cartographer_ros_msgs::PathPlan::Response &res) const;
    bool ReconnectSubmapService(cartographer_ros_msgs::ReconnectSubmaps &req,
                                cartographer_ros_msgs::ReconnectSubmaps &res);
    void PublishPath(const ::ros::WallTimerEvent& timer_event);
    
    // For test and display
    ::ros::Subscriber clicked_point_subscriber_ GUARDED_BY(mutex_);
    void IsClickedPointFree(const geometry_msgs::PointStamped::ConstPtr& msg) const;
    void NavigateToClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg) const;
};
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros
#endif /* node_hpp */
