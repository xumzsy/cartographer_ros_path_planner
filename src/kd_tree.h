/*
 * Author: Mengze Xu
 * Implement 2d kd tree for quick near neighbor search in RRT and submaps.
 *
 * TODO: Add rebalance method and make the clss more unisersal
 */

#ifndef kd_tree_h
#define kd_tree_h

#include <vector>
#include "geometry_msgs/Point.h"

namespace cartographer_ros {
namespace cartographer_ros_path_planner{
    
struct KdTreeNode{
    geometry_msgs::Point point;
    KdTreeNode* parent_node;        // Used for plan planning
    double distance;                // Used for plan planning
    KdTreeNode* left_node;          // Used for kd tree
    KdTreeNode* right_node;         // used for kd tree
    int trajectory_id;              // used for submaps
    int submap_index;               // used for submaps
    
    KdTreeNode();
    KdTreeNode(geometry_msgs::Point p);
    KdTreeNode(geometry_msgs::Point p, int trajectory_idx, int submap_idx);
};    
    
    
class KdTree{
public:
    // Return nearest node to target point
    KdTreeNode* NearestKdTreeNode(const geometry_msgs::Point& target) const;
    
    // Return near nodes around target within radius
    std::vector<KdTreeNode*> NearKdTreeNode(const geometry_msgs::Point& target,
                                            double radius) const;
    
    // Add a new point into RRT tree
    KdTreeNode* AddPointToKdTree(geometry_msgs::Point point);
    KdTreeNode* AddPointToKdTree(geometry_msgs::Point point, KdTreeNode* parent, int depth);
    KdTreeNode* AddPointToKdTree(geometry_msgs::Point point, int trajectory_idx, int submap_idx);

    // Constructor
    KdTree();
    KdTree(geometry_msgs::Point start_point);
    ~KdTree(){DestroyRRT(root_);}

    // For test
    KdTreeNode* BruceNearestKdTreeNode(const geometry_msgs::Point& point);
    std::vector<KdTreeNode*> BruceNearKdTreeNode(const geometry_msgs::Point& target, double radius);
    
private:
    
    // Help function to go through kd tree
    void SearchKdTreeNode(const geometry_msgs::Point& target,
                          KdTreeNode* current_node,
                          KdTreeNode*& current_nearest_node,
                          double& current_cloest_distance2,
                          int depth) const;
    void SearchKdTreeNode(const geometry_msgs::Point& target,
                          KdTreeNode* current_node,
                          std::vector<KdTreeNode*>& near_nodes,
                          double radius,
                          int depth) const;
    // Destroy RRT
    void DestroyRRT(KdTreeNode* root);
    
    KdTreeNode* const root_;
};

} // namespace cartographer_ros_path_planner
} // namespace cartographer_ros

#endif /* kd_tree_h */
