//
//  local_rrt_planner.cpp
//  
//
//
//

#include "local_rrt_planner.h"
namespace cartographer_ros{
namespace cartographer_ros_navigation {

struct RRTreeNode{
    geometry_msgs::Point point;
    RRTreeNode* parent_node;
    std::vector<RRTreeNode*> children_node;
    
    RRTreeNode(){parent_node=nullptr;children_node.clear();};
    RRTreeNode(geometry_msgs::Point p){point=p;parent_node=nullptr;children_node.clear();}
};

    
// Return whether a point is free in a submap
int IsLocalFree(const geometry_msgs::Point& point,   // only use x & y in 2D case
                const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                SubmapIndex submap_index) const {
    const auto& submap_grid_ptr = submap_grids.find(submap_index);;
    if(submap_grid_ptr!=submap_grids.end()){
        const auto& submap_grid = submap_grid_ptr->second;
        const int x = (point.x - submap_grid.x0) / submap_grid.resolution;
        const int y = (point.y - submap_grid.y0) / submap_grid.resolution;
        if(x>=0 && x<submap_grid.width && y>=0 && y<submap_grid.height){
            return submap_grid.data.at(y*submap_grid.width+x);
        } else{
            std::cout<<"Point is out of submap range"<<std::endl;
            return -1;
        }
    } else{
        std::cout<<"Submap "<<submap_index<<" do not exist!"<<std::endl;
        return -1;
    }
}

// Return whether a straight line between two points are free
bool IsPathLocalFree(const geometry_msgs::Point& start,
                     const geometry_msgs::Point& end,
                     const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                     const std::vector<SubmapIndex>& submap_indexs){
    std::cout<<"Begin Check Local Path"<<std::endl;
    float distance2 = Distance2BetweenPoint(start,end);
    float step = kOccupyGridResolution / sqrt(distance2);
    for(float i=0;i<=1;i+=step){
        geometry_msgs::Point point = (1.0-i) * start + i * end;
        bool is_free = false;
        for(auto submap_index:submap_indexs){
            int val = IsLocalFree(point, submap_grids, submap_index);
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
    
// Return the nearest RRTNode to target point
RRTreeNode* NearestRRTreeNode(RRTreeNode* const root, const geometry_msgs::Point& target){
    std::cout<<"Try to find the nearest node in RRT"<<std::endl;
    float min_distance = FLT_MAX;
    RRTreeNode* nearest_node = root;
    std::queue<RRTreeNode*> node_to_visit;
    node_to_visit.push(root);
    while(!node_to_visit.empty()){
        RRTreeNode* current_node = node_to_visit.front();
        //std::cout<<current_node->point<<": parent"<<(current_node->parent_node!=nullptr)<<" ,children: "<<current_node->children_node.size()<<std::endl;
        float distance2 = Distance2BetweenPoint(target,current_node->point);
        if(distance2 < min_distance){
            min_distance = distance2;
            nearest_node = current_node;
        }
        //std::cout<<distance2<<std::endl;
        for(RRTreeNode* node:current_node->children_node ){
            if( current_node->children_node.size()>20) break;
            node_to_visit.push(node);
        }
        //std::cout<<"Push"<<std::endl;
        if(!node_to_visit.empty()) node_to_visit.pop();
        //std::cout<<"PoP"<<std::endl;
    }
    std::cout<<"End to find the nearest node in RRT"<<std::endl;
    return nearest_node;
}

// Genearate a free random point in given submaps
geometry_msgs::Point RandomFreePoint(const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                                     const std::vector<SubmapIndex>& submap_indexes);{
    std::cout<<"Begin Generate Random Point"<<std::endl;
    while(true){
        int random_idx = rand() % submap_indexes.size();
        auto& submap_grid = submap_grids.find(submap_indexes[random_idx])->second;
        int random_x = rand() % submap_grid.width;
        int random_y = rand() % submap_grid.height;
        int val = submap_grid.data[random_y * submap_grid.width + random_x];
        if(val>=0 && val<kOccupyThreshhold){
            std::cout<<"Try to Generate one point"<<std::endl;
            geometry_msgs::Point point;
            point.x = random_x * submap_grid.resolution + submap_grid.x0;
            point.y = random_y * submap_grid.resolution + submap_grid.y0;
            point.z = 0.0;
            return point;
        }
    }
}

// Add a new point into RRT
bool AddNewPoint(geometry_msgs::Point next_point){
    RRTreeNode* nearest_node = NearestRRTreeNode(root,next_point);
    if(!IsPathLocalFree(nearest_node->point,next_point,submap_grids,submap_indexes)) return false;
    RRTreeNode* next_node = new RRTreeNode(next_point);
    next_node->parent_node = nearest_node;
    nearest_node->children_node.push_back(next_node);
    std::cout<<"Successfully Add a new node to RRT"<<std::endl;
    return true;
}
    
// connecting two points in given submaps
Path LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                      const geometry_msgs::Point& end_point,
                      const std::map<SubmapIndex,SubmapGrid>& submap_grids,
                      const std::vector<SubmapIndex> submap_indexs){
    // Naively check the straight line between two points
    if(IsPathLocalFree(start_point,end_point,submap_grids,submap_indexes)){
        std::cout<<"Directly connect two submaps: "<<start_point<<","<<end_point<<std::endl;
        return {start_point,end_point};
    }
    Path path;
    RRTreeNode* root = new RRTreeNode (start_point);
    for(int node_num=0;node_num<kMaxRRTNodeNum;node_num++){
        geometry_msgs::Point next_point;
        if((rand() % 1000) / 1000.0 < kProbabilityOfChooseEndPoint){
            // choose end point
            next_point = (end_point);
        } else{
            // random generate point
            next_point =  RandomFreePoint(submap_grids,submap_indexes);
        }
        // TODO: mid point of next point
        
        if(!AddNewPoint(next_point)){node_num--;continue;}
        
        // try to connect RRT and end point
        if(node_num % kStepToCheckReachEndPoint == 0){
            std::cout<<"Try to connect End point! Node Num:"<<node_num<<std::endl;
            RRTreeNode* node = NearestRRTreeNode(root,end_point);
            if(IsPathLocalFree(node->point,end_point,submap_grids,submap_indexes)){
                // find the path!
                while(node!=nullptr){
                    path.insert(path.begin(),node->point);
                    node = node->parent_node;
                }
                path.push_back(end_point);
                std::cout<<"Successfully find a path!"<<std::endl;
                DestroyRRTree(root);
                return path;
            }
        }
    }
    std::cout<<"Fail to find a path from submap!"<<std::endl;
    DestroyRRTree(root);
    return {};
}
    
// destropy the RRT tree
void DestroyRRTree(RRTreeNode* root){
    if(root!=nullptr){
        for(auto& child:root->children_node){
            DestroyRRTree(child);
        }
        delete(root);
    }
}
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros
