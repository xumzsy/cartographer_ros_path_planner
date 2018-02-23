/*
 
 
 */

#include <queue>
#include <float.h>

#include "common.h"
#include "kd_tree.h"

namespace cartographer_ros {
namespace cartographer_ros_navigation{

// Return nearest node to target
KdTreeNode* KdTree::NearestKdTreeNode(const geometry_msgs::Point& target) const {
    KdTreeNode* nearest_node = nullptr;
    float closest_distance2 = FLT_MAX;
    SearchKdTreeNode(target, root_, nearest_node, closest_distance2,0);
    return nearest_node;
}
    
// Return near nodes around target within radius
std::vector<KdTreeNode*> KdTree::NearKdTreeNode(const geometry_msgs::Point& target,
                                                float radius) const {
    std::vector<KdTreeNode*> near_nodes;
    SearchKdTreeNode(target,root_,near_nodes,radius * radius,0);
    return near_nodes;
}

// Helper function to go through KdTree
void KdTree::SearchKdTreeNode(const geometry_msgs::Point& target,
                              KdTreeNode* current_node,
                              KdTreeNode*& current_nearest_node,
                              float& current_cloest_distance2,
                              int depth) const{
    if(current_node==nullptr) return;
    float distance2 = Distance2BetweenPoint(target, current_node->point);
    bool go_to_left = depth%2==0 ? target.x <= current_node->point.x   // Compare x
    : target.y <= current_node->point.y;  // Compare y
    // Recursively visit children
    go_to_left ?
    SearchKdTreeNode(target, current_node->left_node,
                     current_nearest_node,current_cloest_distance2, depth+1) :
    SearchKdTreeNode(target, current_node->right_node,
                     current_nearest_node,current_cloest_distance2, depth+1);
    
    // Search current_node
    if(distance2<current_cloest_distance2){
        current_cloest_distance2 = distance2;
        current_nearest_node = current_node;
    }
    
    // Judge whether search other side
    float discard_threshold = depth%2==0
    ?   (target.x - current_node->point.x) * (target.x - current_node->point.x)
    :   (target.y - current_node->point.y) * (target.y - current_node->point.y);
    if(current_cloest_distance2 > discard_threshold){
        go_to_left ? SearchKdTreeNode(target,current_node->right_node,
                                      current_nearest_node,current_cloest_distance2,depth+1)
        :   SearchKdTreeNode(target,current_node->left_node,
                             current_nearest_node,current_cloest_distance2,depth+1);
    }
}

// Helper function to go through KdTree
void KdTree::SearchKdTreeNode(const geometry_msgs::Point& target,
                              KdTreeNode* current_node,
                              std::vector<KdTreeNode*>& near_nodes,
                              float radius,
                              int depth) const{
    if(current_node==nullptr) return;
    float distance2 = Distance2BetweenPoint(target, current_node->point);
    // Visit current node
    if(distance2 < radius) near_nodes.push_back(current_node);
    
    // Recursively visit children
    bool go_to_left = depth%2==0 ? target.x <= current_node->point.x   // Compare x
                                 : target.y <= current_node->point.y;  // Compare y
    if(go_to_left){
        SearchKdTreeNode(target, current_node->left_node, near_nodes,radius, depth+1);
    } else{
        SearchKdTreeNode(target, current_node->right_node, near_nodes,radius, depth+1);
    }
    
    // Visit the otherside
    float discard_threshold = depth%2==0
    ?   (target.x - current_node->point.x) * (target.x - current_node->point.x)
    :   (target.y - current_node->point.y) * (target.y - current_node->point.y);
    if(radius > discard_threshold){
        go_to_left ? SearchKdTreeNode(target, current_node->right_node, near_nodes, radius, depth+1)
                   : SearchKdTreeNode(target, current_node->left_node, near_nodes,radius, depth+1);
    }
}

// Add a new point into RRT tree and return a pointer to it
KdTreeNode* KdTree::AddPointToKdTree(geometry_msgs::Point point){
    return AddPointToKdTree(point, root_, 0);
}

// Recursively add the point into kd tree
KdTreeNode* KdTree::AddPointToKdTree(geometry_msgs::Point point, KdTreeNode* parent, int depth){
    bool go_to_left = depth%2==0 ? point.x <= parent->point.x   // Compare x
                                 : point.y <= parent->point.y;  // Compare y
    if(go_to_left){
        if(parent->left_node==nullptr){
            parent->left_node = new KdTreeNode(point);
            return parent->left_node;
        } else{
            AddPointToKdTree(point, parent->left_node, depth+1);
        }
    } else{
        if(parent->right_node==nullptr){
            parent->right_node = new KdTreeNode(point);
            return parent->right_node;
        } else{
            AddPointToKdTree(point, parent->right_node, depth+1);
        }
    }
}

// Constructor
KdTree::KdTree(geometry_msgs::Point start_point) : root_(new KdTreeNode (start_point)){}

// Destroy RRT
void KdTree::DestroyRRT(KdTreeNode* root){
    if(root!=nullptr){
        DestroyRRT(root->left_node);
        DestroyRRT(root->right_node);
        delete(root);
    }
}

// For test
KdTreeNode* KdTree::BruceNearestKdTreeNode(const geometry_msgs::Point& target){
    KdTreeNode* nearest_node = nullptr;
    float min_distance2 = FLT_MAX;
    std::queue<KdTreeNode*> q;
    q.push(root_);
    while(!q.empty()){
        float distance2 = Distance2BetweenPoint(target, q.front()->point);
        if(distance2<min_distance2){
            min_distance2 = distance2;
            nearest_node = q.front();
        }
        if(q.front()->left_node) q.push(q.front()->left_node);
        if(q.front()->right_node) q.push(q.front()->right_node);
        q.pop();
    }
    return nearest_node;
}


std::vector<KdTreeNode*> KdTree::BruceNearKdTreeNode(const geometry_msgs::Point& target,
                                                              float radius){
    std::vector<KdTreeNode*> bruce_near_nodes;
    std::queue<KdTreeNode*> q;
    q.push(root_);
    while(!q.empty()){
        float distance2 = Distance2BetweenPoint(target, q.front()->point);
        if(distance2 < radius){
            bruce_near_nodes.push_back(q.front());
        }
        if(q.front()->left_node) q.push(q.front()->left_node);
        if(q.front()->right_node) q.push(q.front()->right_node);
        q.pop();
    }
    return bruce_near_nodes;
}
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros

