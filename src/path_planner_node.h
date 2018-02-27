/*
 */

#ifndef PATH_PLANNER_NODE_H
#define PATH_PLANNER_NODE_H

#include <map>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/id.h"
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
namespace cartographer_ros_path_planner {

using ::cartographer::mapping::SubmapId;
using SubmapIndex = int;
using Path = std::vector<geometry_msgs::Point>;
    
// Parameters obtained from ROS param server
struct Parameters{
    int max_rrt_node_num;
    int step_to_check_reach_endpoint;
    double distance_threshold_for_adding;
    double distance2_threshold_for_updating;
    double rotation_threshold_for_updating;
    double probability_of_choose_endpoint;
    double rrt_grow_step;
    double rrt_trim_radius;
};
    
// Core Node provide path plan for cartographer
class PathPlannerNode{
public:
    
    PathPlannerNode();
    ~PathPlannerNode(){};
    
    PathPlannerNode(const PathPlannerNode&) = delete;
    PathPlannerNode& operator=(const PathPlannerNode&) = delete;

    // Get Parameters from ROS param server
    void SetParameters();
    
    
    // Given a point in global map frame and its correspond submap id,
    // return the intensity of the point in submap.
    // val: -1 for unobserved or out of submap range
    //       0 for definitely free and 100 for definitely occupied.
    int GetPointIntensity(const geometry_msgs::Point& point,
                          const SubmapId submap_id) const;
    
    
    // Return whether a point in global map frame is free or not
    bool IsFree(const geometry_msgs::Point& point) const;
    
    // Return whether the straight line between two points are free or not
    // in given submaps
    bool IsPathLocalFree(const geometry_msgs::Point& start_point,
                         const geometry_msgs::Point& end_point,
                         const std::vector<SubmapId>& submap_ids) const;
    
    // Return the closest submap to given point
    SubmapId ClosestSubmap(const geometry_msgs::Point& point) const;
    
    // Return a list of submaps whose origin is within a circle of radius
    // to given point
    // TODO: use centre instead of origin
    std::vector<SubmapId> CloseSubmaps(const geometry_msgs::Point& point,
                                       double radius) const;
    
    // Return a free path from start point to end point using RRT*
    Path PlanPathRRT(const geometry_msgs::Point& start,
                     const geometry_msgs::Point& end);
    
    // Return a free path between two submaps' origin using RRT*
    Path PlanPathRRT(const SubmapId& start_id, const SubmapId& end_id);
    
    // Returan a path connecting two remote submaps using graph search
    Path ConnectSubmap(const SubmapId& start_id, const SubmapId& end_id);
    
    // Return a path between two points. It's' supposed that the submaps
    // that the points are located are directly connected
    Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                          const geometry_msgs::Point& end_point);
    
    // Return a path between two points in given submaps
    Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                          const geometry_msgs::Point& end_point,
                          const std::vector<SubmapId> submap_ids);
    
    // Publish and display the path in RVIZ
    void AddDisplayPath(Path path);
    
private:
    struct SubmapConnectState{
        SubmapId start_submap_id;
        SubmapId end_submap_id;
        double length;
        Path path;
        
        SubmapConnectState(){};
        SubmapConnectState(SubmapId start_id, SubmapId end_id, double d) :
        start_submap_id(start_id),
        end_submap_id(end_id),
        length(d) {};
    };
    
    struct SubmapGrid{
        SubmapId submap_id;
        double resolution;
        double x0;
        double y0;
        int width;
        int height;
        std::vector<int> data;
    };
    
    // Generate random free point in given submaps used in RRT*
    geometry_msgs::Point RandomFreePoint(const std::vector<SubmapId>& submap_ids) const;
    
    // Add submap grid into submap_grid_
    void AddSubmapGrid(const SubmapId& submap_id);
    
    // Discard existing path if existed and reconnect two submaps
    // Return false if fail to connect two submaps
    bool ReconnectSubmaps(const SubmapId& start_id, const SubmapId& end_id);
    
    // Add a submap into road_map_
    void AddRoadMapEntry(const SubmapId& submap_id);
    
    // Update submap_, road_map_ and submap_grid_ every time receive submapList
    void UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
    
    // ROS Agent
    ::cartographer::common::Mutex mutex_;
    ::ros::NodeHandle node_handle_ GUARDED_BY(mutex_);
    ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
    ::ros::ServiceClient submap_query_client_ GUARDED_BY(mutex_);
    Parameters parameters_;
    
    // SubmapList
    std::map<SubmapId,cartographer_ros_msgs::SubmapEntry> submap_ GUARDED_BY(mutex_);
    
    // Kd tree for Submap position
    KdTree submap_kdtree_;
    
    // SubmapGrid
    std::map<SubmapId, SubmapGrid> submap_grid_ GUARDED_BY(mutex_);
    
    // Road map to store the connectivity of submaps and corresponding path
    std::map<SubmapId,std::map<SubmapId,SubmapConnectState>> road_map_ GUARDED_BY(mutex_);
    
    // ROS Server
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
    
    // Members to publish and display newest path
    nav_msgs::Path path_to_display_;
    ::ros::WallTimer path_publisher_timer_;
    ::ros::Publisher path_publisher_;
    void PublishPath(const ::ros::WallTimerEvent& timer_event);
    
    // print out the current state for testing and debugging
    
    ::ros::Subscriber clicked_point_subscriber_ GUARDED_BY(mutex_);
    
    void IsClickedPointFree(const geometry_msgs::PointStamped::ConstPtr& msg) const;
    void NavigateToClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg);
};
    
} // namespace cartographer_ros_path_planner
} // namespace cartographer_ros
#endif /* path_planner_node.h */
