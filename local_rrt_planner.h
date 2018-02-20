//
//  local_rrt_planner.hpp
//  
//
//  
//

#ifndef local_rrt_planner_h
#define local_rrt_planner_h

#include "navigation_node.h"
#include "utils.h"

namespace cartographer_ros{
namespace cartographer_ros_navigation {
namespace{
    
} // namespace
    
struct RRTreeNode{
    geometry_msgs::Point point;
    RRTreeNode* parent_node;
    std::vector<RRTreeNode*> children_node;
    RRTreeNode(){parent_node=nullptr;children_node.clear();};
    RRTreeNode(geometry_msgs::Point p){point=p;parent_node=nullptr;children_node.clear();}
};

// Return the val of point in given submap
int IsLocalFree(const geometry_msgs::Point& point,   // only use x & y in 2D case
                const std::map<SubmapIndex,SubmapGrid>& submap_grids
                SubmapIndex submap_index) const;
    
// Return whether a straight line between two points are free
bool IsPathLocalFree(const geometry_msgs::Point& start,
                     const geometry_msgs::Point& end,
                     const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                     const std::vector<SubmapIndex>& submap_indexs);
    
// Return the nearest RRTNode to target point
RRTreeNode* NearestRRTreeNode(RRTreeNode* root, const geometry_msgs::Point& target);
    
// Add a new point into RRT
bool AddNewPoint(geometry_msgs::Point next_point);
    
// Genearate a free random point in given submaps
geometry_msgs::Point RandomFreePoint(const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                                     const std::vector<SubmapIndex>& submap_indexes);
    
// connecting two points in given submaps
Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                      const geometry_msgs::Point& end_point,
                      const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                      const std::vector<SubmapIndex> submap_indexs);

// Function to destroy RRTreeNode
void DestroyRRTree(RRTreeNode* root);

} // namespace cartographer_ros_navigation
} // namespace cartographer_ros


#endif /* local_rrt_planner_h */
