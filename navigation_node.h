//
//  node.h
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
#include "ros/serialization.h"
#include "kd_tree.h"

namespace cartographer_ros{
namespace cartographer_ros_navigation {

using SubmapIndex = int;
using Path = std::vector<geometry_msgs::Point>;
    
// parameters
struct Parameters{
    int max_rrt_node_num;
    int step_to_check_reach_endpoint;
    double distance2_threshold_for_adding;
    double distance2_threshold_for_updating;
    double rotation_threshold_for_updating;
    double probability_of_choose_endpoint;
    double rrt_grow_step;
    double rrt_trim_radius;
};
    
// Node to provide navigation for cartographer
class NavigationNode{
public:
    
    NavigationNode();
    ~NavigationNode(){};
    
    NavigationNode(const NavigationNode&) = delete;
    NavigationNode& operator=(const NavigationNode&) = delete;

    // Set Parameters
    void SetParameters();
    
    // Return whether a point is free in local submap
    bool IsFree(const geometry_msgs::Point& point) const;
    bool IsPathFree(const geometry_msgs::Point& start_point,
                    const geometry_msgs::Point& end_point) const;
    int IsLocalFree(const geometry_msgs::Point& point, SubmapIndex submap_index) const;
    bool IsPathLocalFree(const geometry_msgs::Point& start_point,
                         const geometry_msgs::Point& end_point,
                         const std::vector<SubmapIndex>& submap_indexs) const;
    
    // Return the cloest SubmapIndex if pose is free in this submap
    SubmapIndex CloestSubmap(const geometry_msgs::Point& point, double radius) const;
    std::vector<SubmapIndex> CloseSubmaps(const geometry_msgs::Point& point) const;
    
    // Return a free path from starting position to end postion using RRT
    Path PlanPathRRT(const geometry_msgs::Point& start,
                     const geometry_msgs::Point& end);
    
    // Return a free path between submaps' origin using RRT
    Path PlanPathRRT(SubmapIndex start_idx, SubmapIndex end_idx);
    
    // Returan a path connecting two remote submaps using graph search
    Path ConnectingSubmap(SubmapIndex start_idx, SubmapIndex end_idx);
    
    // connecting two points in given submaps
    Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                          const geometry_msgs::Point& end_point,
                          const std::vector<SubmapIndex> submap_indexs);
    
    // Generate random free point in given submaps
    geometry_msgs::Point RandomFreePoint(const std::vector<SubmapIndex>& submap_indexes);
    
    // print out the current state for testing and debugging
    void AddDisplayPath(Path path);
    
private:
    struct SubmapConnectState{
        SubmapIndex start_submap_index;
        SubmapIndex end_submap_index;
        double length;
        Path path;
        
        SubmapConnectState(){};
        SubmapConnectState(SubmapIndex start_index, SubmapIndex end_index, double d) :
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
    Parameters parameters_;
    ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
    ::ros::ServiceClient submap_query_client_ GUARDED_BY(mutex_);
    
    // SubmapList
    std::map<SubmapIndex,cartographer_ros_msgs::SubmapEntry> submap_ GUARDED_BY(mutex_);
    
    // Kd tree for Submap position
    KdTree submap_kdtree_;
    
    // SubmapGrid
    std::map<SubmapIndex, SubmapGrid> submap_grid_ GUARDED_BY(mutex_);
    
    // Road map to store the connectivity of submaps and corresponding path
    std::map<SubmapIndex,std::map<SubmapIndex,SubmapConnectState>> road_map_ GUARDED_BY(mutex_);
    
    // ROS Server
    nav_msgs::Path path_to_display_;
    ::ros::WallTimer path_publisher_timer_;
    ::ros::Publisher path_publisher_;
    ::ros::ServiceServer roadmap_query_server_;
    ::ros::ServiceServer connection_query_server_;
    ::ros::ServiceServer plan_path_server_;
    ::ros::ServiceServer reconnect_submaps_server_;
    bool QueryRoadmap(cartographer_ros_msgs::RoadmapQuery::Request &req,
                      cartographer_ros_msgs::RoadmapQuery::Response &res);
    bool QueryConnection(cartographer_ros_msgs::ConnectionQuery::Request &req,
                         cartographer_ros_msgs::ConnectionQuery::Response &res);
    bool PlanPath(cartographer_ros_msgs::PathPlan::Request &req,
                  cartographer_ros_msgs::PathPlan::Response &res);
    bool ReconnectSubmapService(cartographer_ros_msgs::ReconnectSubmaps::Request &req,
                                cartographer_ros_msgs::ReconnectSubmaps::Response &res);
    void PublishPath(const ::ros::WallTimerEvent& timer_event);
    
    // For test and display
    ::ros::Subscriber clicked_point_subscriber_ GUARDED_BY(mutex_);
    void IsClickedPointFree(const geometry_msgs::PointStamped::ConstPtr& msg) const;
    void NavigateToClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg);
};
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros
#endif /* node_hpp */
