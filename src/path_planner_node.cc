/*
 * Author: Mengze Xu
 */


#include <math.h>
#include <queue>
#include <string>
#include <set>
#include <stdlib.h>
#include <time.h>
#include <set>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/submap.h"

#include "common.h"
#include "path_planner_node.h"


namespace cartographer_ros{
namespace cartographer_ros_path_planner {
namespace{
    
using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;

const char kSubmapListTopicName [] = "/submap_list";
const char kSubmapQueryServiceName [] = "/submap_query";
const char kRoadmapQueryServiceName [] = "/roadmap_query";
const char kConnectionQueryServiceName [] = "/connection_query";
const char kReconnectSubmapsServiceName [] = "/reconnect_submaps";
const char kPathPlanServiceName [] = "/plan_path";
const int kFinishVersion = 180;
const int kOccupyThreshold = 64;
const double kOccupyGridResolution = 0.05;
const double kProbabilityGridResolution = 0.05;
    
// Define add and scalar multiplication for Point
geometry_msgs::Point operator+(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    geometry_msgs::Point sum;
    sum.x = a.x + b.x;
    sum.y = a.y + b.y;
    sum.z = a.z + b.z;
    return sum;
}

geometry_msgs::Point operator*(double a, const geometry_msgs::Point& b){
    geometry_msgs::Point product;
    product.x = a * b.x;
    product.y = a * b.y;
    product.z = a * b.z;
    return product;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point& point){
    os<<" ("<<point.x<<","<<point.y<<","<<point.z<<") ";
    return os;
}

std::ostream& operator<<(std::ostream& os, Path& path){
    for(geometry_msgs::Point& point: path){
        os<<point<<std::endl;
    }
    return os;
}

double RotationBetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    return abs(pose1.orientation.z-pose1.orientation.z);
}

// Judge intensity val free or not
bool IsValueFree(int val){
    return 0<=val && val<kOccupyThreshold;
}
    
// Calculate L2 length of the path
double LengthOfPath(const Path& path){
    double length = 0.0;
    for(size_t i=0;i<path.size()-1;i++){
        double distance2 = Distance2BetweenPoint(path[i],path[i+1]);
        length += sqrt(distance2);
    }
    return length;
}
  
} // namespace


PathPlannerNode::PathPlannerNode(){
    cartographer::common::MutexLocker lock(&mutex_);
    submap_list_subscriber_ = node_handle_.subscribe<cartographer_ros_msgs::SubmapList>(kSubmapListTopicName,10, &PathPlannerNode::UpdateRoadMap,this);
    submap_query_client_ = node_handle_.serviceClient<cartographer_ros_msgs::SubmapQuery>(kSubmapQueryServiceName);
    roadmap_query_server_ = node_handle_.advertiseService(kRoadmapQueryServiceName,&PathPlannerNode::QueryRoadmap,this);
    connection_query_server_ = node_handle_.advertiseService(kConnectionQueryServiceName, &PathPlannerNode::QueryConnection,this);
    plan_path_server_ = node_handle_.advertiseService(kPathPlanServiceName,&PathPlannerNode::PlanPath,this);
    reconnect_submaps_server_ = node_handle_.advertiseService(kReconnectSubmapsServiceName, &PathPlannerNode::ReconnectSubmapService,this);
    path_publisher_ = node_handle_.advertise<::nav_msgs::Path>("/test_path", 10);
    path_publisher_timer_ = node_handle_.createWallTimer(::ros::WallDuration(0.1), &PathPlannerNode::PublishPath, this);
    SetParameters();
    srand (time(NULL));
    
    // For test
    clicked_point_subscriber_ = node_handle_.subscribe<geometry_msgs::PointStamped>("/clicked_point",1,&PathPlannerNode::NavigateToClickedPoint, this);
    std::cout<<"Successfully Create PathPlannerNode"<<std::endl;
}

    
void PathPlannerNode::SetParameters(){
    if(!node_handle_.getParam("max_rrt_node_num" ,parameters_.max_rrt_node_num)){
        std::cout<<"Error! Fail to get paramter: max_rrt_node_num"<<std::endl;
    }
    if(!node_handle_.getParam("step_to_check_reach_endpoint" ,parameters_.step_to_check_reach_endpoint)){
        std::cout<<"Error! Fail to get paramter: step_to_check_reach_endpoint "<<std::endl;
    }
    if(!node_handle_.getParam("distance_threshold_for_adding" ,parameters_.distance_threshold_for_adding)){
        std::cout<<"Error! Fail to get paramter: distance_threshold_for_adding"<<std::endl;
    }
    if(!node_handle_.getParam("distance_threshold_for_updating" ,parameters_.distance2_threshold_for_updating)){
        std::cout<<"Error! Fail to get paramter: distance_threshold_for_updating"<<std::endl;
    }
    parameters_.distance2_threshold_for_updating *= parameters_.distance2_threshold_for_updating;
    if(!node_handle_.getParam("rotation_threshold_for_updating" ,parameters_.rotation_threshold_for_updating)){
        std::cout<<"Error! Fail to get paramter: rotation_threshold_for_updating"<<std::endl;
    }
    if(!node_handle_.getParam("probability_of_choose_endpoint" ,parameters_.probability_of_choose_endpoint)){
        std::cout<<"Error! Fail to get paramter: probability_of_choose_endpoint"<<std::endl;
    }
    if(!node_handle_.getParam("rrt_grow_step" ,parameters_.rrt_grow_step)){
        std::cout<<"Error! Fail to get paramter: rrt_grow_step"<<std::endl;
    }
    if(!node_handle_.getParam("rrt_trim_radius" ,parameters_.rrt_trim_radius)){
        std::cout<<"Error! Fail to get paramter: rrt_trim_radius"<<std::endl;
    }
    if(!node_handle_.getParam("close_submap_radius" ,parameters_.close_submap_radius)){
        std::cout<<"Error! Fail to get paramter: close_submap_radius"<<std::endl;
    }


}
    
    
int PathPlannerNode::GetPointIntensity(const geometry_msgs::Point& point,
                                       const SubmapId& submap_id) const{
    if(submap_grid_.count(submap_id)==1){
        const auto& submap_grid = submap_grid_.find(submap_id)->second;
        int x = (point.x - submap_grid.x0) / submap_grid.resolution;
        int y = (point.y - submap_grid.y0) / submap_grid.resolution;
        if(x>=0 && x<submap_grid.width && y>=0 && y<submap_grid.height){
            int val = submap_grid.data[y*submap_grid.width+x];
            return val;
        } else{
            return -1;
        }
    } else{
        std::cout<<"Submap "<<submap_id<<" not exist!"<<std::endl;
        return -1;
    }
}
    
    
bool PathPlannerNode::IsFree(const geometry_msgs::Point& point) const{
    auto close_submaps = CloseSubmaps(point, parameters_.close_submap_radius);
    bool is_free = false;
    for(const auto& submap_id:close_submaps){
        int val = GetPointIntensity(point,submap_id);
        if(val>=kOccupyThreshold) return false;
        if(val>=0) is_free = true;
    }
    return is_free;
}
    
    
bool PathPlannerNode::IsPathLocalFree(const geometry_msgs::Point& start_point,
                                      const geometry_msgs::Point& end_point,
                                      const std::vector<SubmapId>& submap_ids) const {
    double distance2 = Distance2BetweenPoint(start_point,end_point);
    double step = kOccupyGridResolution / sqrt(distance2);
    for(double i=0;i<=1;i+=step){
        geometry_msgs::Point point = (1.0-i) * start_point + i * end_point;
        bool is_free = false;
        for(const auto& submap_id:submap_ids){
            int val = GetPointIntensity(point, submap_id);
            if(val>=kOccupyThreshold) return false;
            if(val>=0) is_free = true;
        }
        if(!is_free) return false;
    }
    return true;
}

    
SubmapId PathPlannerNode::ClosestSubmap(const geometry_msgs::Point& point) const{
    const auto nearest_node = submap_kdtree_.NearestKdTreeNode(point);
    return SubmapId {nearest_node->trajectory_id,nearest_node->submap_index};
}

    
std::vector<SubmapId> PathPlannerNode::CloseSubmaps(const geometry_msgs::Point& point,
                                                    double radius) const{
    const auto& near_nodes = submap_kdtree_.NearKdTreeNode(point, radius);
    std::vector<SubmapId> close_submaps;
    close_submaps.reserve(near_nodes.size());
    for(const auto near_node:near_nodes){
        SubmapId submap_id {near_node->trajectory_id,near_node->submap_index};
        close_submaps.push_back(std::move(submap_id));
    }
    return close_submaps;
}
    
    
Path PathPlannerNode::PlanPathRRT(const geometry_msgs::Point& start_point,
                                  const geometry_msgs::Point& end_point) const {
    // Check start and end point is free
    if(!IsFree(start_point)){
        std::cout<<"Start point is occupied!"<<std::endl;
        return {};
    }
    if(!IsFree(end_point)){
        std::cout<<"End point is occupied!"<<std::endl;
        return {};
    }
    std::cout<<"Begin to plan a path from"<<start_point<<"to"<<end_point<<std::endl;
    Path path;
    
    SubmapId start_submap_id = ClosestSubmap(start_point);
    SubmapId end_submap_id = ClosestSubmap(end_point);
    std::cout<<"Locate start point at submap"<<start_submap_id<<std::endl;
    std::cout<<"Locate end point at submap"<<end_submap_id<<std::endl;
    
    // If start and end point are in the same submap, directly plan path in the submap
    if(start_submap_id==end_submap_id){
        path = LocalPlanPathRRT(start_point,end_point);
        return path;
    }
    
    // Connect start point and the origin of start submap
    Path start_path = LocalPlanPathRRT(start_point,submap_.find(start_submap_id)->second.pose.position);
    if(!start_path.empty()){
        path.insert(path.end(),start_path.begin(),start_path.end());
    } else{
        std::cout<<"Fail to connect start point to start_submap"<<std::endl;
        return {};
    }
    
    // Connect start_submap to end_submap in road_map_
    auto mid_path = ConnectSubmap(start_submap_id,end_submap_id);
    if(!mid_path.empty()){
        path.insert(path.end(),mid_path.begin(),mid_path.end());
    } else{
        std::cout<<"Fail to connect submap "<<start_submap_id<<" to submap "<<end_submap_id<<std::endl;
        return {};
    }
    
    // Connect end_submap to end point
    auto end_path = LocalPlanPathRRT(submap_.find(end_submap_id)->second.pose.position,end_point);
    if(!end_path.empty()){
        path.insert(path.end(),end_path.begin(),end_path.end());
    } else{
        std::cout<<" Fail to connect end point to end_submap"<<std::endl;
        return {};
    }
    std::cout<<"Successfully find a path!"<<std::endl;
    return path;
}
    
    
Path PathPlannerNode::PlanPathRRT(const SubmapId& start_id, const SubmapId& end_id) const {
    std::cout<<"Try to connect two submaps:"<<start_id<<","<<end_id<<std::endl;
    const auto start_submap = submap_.find(start_id);
    const auto end_submap = submap_.find(end_id);
    if(start_submap==submap_.end()){
        std::cout<<"Submap "<<start_id<<" not exist!"<<std::endl;
        return {};
    }
    if(end_submap==submap_.end()){
        std::cout<<"Submap "<<end_id<<" not exist!"<<std::endl;
        return {};
    }

    geometry_msgs::Point start_point = start_submap->second.pose.position;
    geometry_msgs::Point end_point = end_submap->second.pose.position;
    Path path = LocalPlanPathRRT(start_point, end_point);
    if(!path.empty()){
        std::cout<<"Successfully connect submap "<<start_id<<" and "<<end_id<<std::endl;
    } else{
        std::cout<<"Fail to connect submap "<<start_id<<" and "<<end_id<<std::endl;
    }
    return path;
}

    
    struct CustomPriorityCompare{
        CustomPriorityCompare(std::map<SubmapId, double>& visited_submap_distance):submap_distance(visited_submap_distance) {}
        bool operator ()(SubmapId a, SubmapId b){
            return submap_distance[a]>submap_distance[b];
        }
    private:
        std::map<SubmapId, double>& submap_distance;
    }
// Use BFS to connect two remote submaps
Path PathPlannerNode::ConnectSubmap(const SubmapId& start_id, const SubmapId& end_id) const {
    std::map<SubmapId, double> visited_submap_distance;
    std::map<SubmapId, SubmapId> previous_submap;
    std::priority_queue<SubmapId,std::vector<SubmapId>,
                        CustomPriorityCompare(visited_submap_distance)> submap_to_visit;
    
    for(const auto& pair:submap_) visited_submap_distance[pair.first] = DBL_MAX;
    visited_submap_distance[start_id] = 0.0;
    submap_to_visit.push(start_id);
    bool find_end_id = false;
    
    while(!submap_to_visit.empty()&&!find_end_id){
        auto current_submap = submap_to_visit.top();
        auto& current_connections = road_map_.find(current_submap)->second;
        for(const auto& entry:current_connections){
            if(entry.second.length + visited_submap_distance[current_submap] <
               visited_submap_distance[entry.first]){
                visited_submap_distance[entry.first] = entry.second.length + visited_submap_distance[current_submap];
                previous_submap[entry.first] = current_submap;
                submap_to_visit.push(entry.first);
                if(entry.first==end_id) {find_end_id = true;break;}
            }
        }
        submap_to_visit.pop();
    }
    
    if(previous_submap.count(end_id)==0){
        return {};
    }
    Path path;
    SubmapId id = end_id;
    while(previous_submap.count(id)==1){
        SubmapId previous_id = previous_submap[id];
        if(previous_submap.count(previous_id)==1){
            auto connection = road_map_.find(previous_id)->second.find(id)->second;
            path.insert(path.begin(),connection.path.begin(),connection.path.end());
        }
        id = previous_submap[id];
    }
    return path;
}
    
    
// Use RRT* to do local planning
Path PathPlannerNode::LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                                       const geometry_msgs::Point& end_point) const{
    SubmapId start_submap_id = ClosestSubmap(start_point);
    SubmapId end_submap_id = ClosestSubmap(end_point);
    
    // Find local submaps used in later planning
    auto start_submap_ids = CloseSubmaps(start_point, parameters_.close_submap_radius);
    auto end_submap_ids = CloseSubmaps(end_point, parameters_.close_submap_radius);
    std::set<SubmapId> union_submap_ids;
    for(SubmapId& submap_id:start_submap_ids){
        union_submap_ids.insert(submap_id);
    }
    for(SubmapId& submap_id:end_submap_ids){
        union_submap_ids.insert(submap_id);
    }
    union_submap_ids.insert(start_submap_id);
    union_submap_ids.insert(end_submap_id);
    std::vector<SubmapId> submap_ids (union_submap_ids.begin(),union_submap_ids.end());
    
    // Call local planner
    return LocalPlanPathRRT(start_point,end_point,submap_ids);
}

    
Path PathPlannerNode::LocalPlanPathRRT(const geometry_msgs::Point& start_point,
                                       const geometry_msgs::Point& end_point,
                                       const std::vector<SubmapId>& submap_ids) const {
    // Naively check the straight line between two points
    if(IsPathLocalFree(start_point,end_point,submap_ids)) return {start_point,end_point};
    
    // Build RRT using kd tree
    KdTree kd_tree (start_point);
    for(int node_num=1;node_num<=parameters_.max_rrt_node_num;node_num++){
        // Generate random point in given submaps
        // Better sample method can accerate the convergence
        geometry_msgs::Point next_point;
        if((rand() % 1000) / 1000.0 < parameters_.probability_of_choose_endpoint){
            next_point = (end_point);
        } else{
            next_point =  RandomFreePoint(submap_ids);
        }
        
        // Search the nearest tree node
        KdTreeNode* nearest_node = kd_tree.NearestKdTreeNode(next_point);
        
        // Or go ahead a given distance
        next_point = (parameters_.rrt_grow_step)*next_point + (1-parameters_.rrt_grow_step)*nearest_node->point;
        
        if(!IsPathLocalFree(nearest_node->point, next_point, submap_ids)){node_num--;continue;}
        // Add next_node into RRT
        KdTreeNode* next_node = kd_tree.AddPointToKdTree(next_point);
        if(next_node==nullptr) std::cout<<"Fail to Add New Node"<<std::endl;
        next_node->distance = nearest_node->distance + sqrt(Distance2BetweenPoint(next_point, nearest_node->point));
        next_node->parent_node = nearest_node;
        
        // Trim RRT
        auto near_nodes = kd_tree.NearKdTreeNode(next_point, parameters_.rrt_trim_radius);
        for(auto& near_node : near_nodes){
            double edge_length = sqrt(Distance2BetweenPoint(next_point, near_node->point));
            if(edge_length + next_node->distance < near_node->distance &&
               IsPathLocalFree(near_node->point,next_point,submap_ids)){
                near_node->parent_node = next_node;
                near_node->distance = edge_length + next_node->distance;
            }
        }
        
        // Try to connect RRT and end point
        if(node_num % parameters_.step_to_check_reach_endpoint == 0){
            std::cout<<"Try to connect End point! Node Num:"<<node_num<<std::endl;
            const KdTreeNode* path_node = kd_tree.NearestKdTreeNode(end_point);
            if(IsPathLocalFree(path_node->point,end_point,submap_ids)){
                // find the path!
                Path path;
                while(path_node!=nullptr){
                    path.insert(path.begin(),path_node->point);
                    path_node = path_node->parent_node;
                }
                path.push_back(end_point);
                return path;
            }
        }
    }
    return {};
}
    
    
void PathPlannerNode::AddDisplayPath(const Path& path){
    path_to_display_.header.stamp = ::ros::Time::now();
    path_to_display_.header.frame_id = "/map";
    path_to_display_.poses.clear();
    for(auto& point : path){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ::ros::Time::now();
        pose_stamped.header.frame_id = "/map";
        pose_stamped.pose.position = std::move(point);
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        path_to_display_.poses.push_back(std::move(pose_stamped));
    }
}
    
    
// Return a random free point in given submaps
geometry_msgs::Point PathPlannerNode::RandomFreePoint(const std::vector<SubmapId>& submap_ids) const {
    while(true){
        int random_id = rand() % submap_ids.size();
        auto& submap_grid = submap_grid_.find(submap_ids[random_id])->second;
        int random_x = rand() % (submap_grid.width);
        int random_y = rand() % (submap_grid.height);
        int val = submap_grid.data[random_y * submap_grid.width + random_x];
        if(val>=0 && val<kOccupyThreshold){
            geometry_msgs::Point point;
            point.x = random_x * submap_grid.resolution + submap_grid.x0;
            point.y = random_y * submap_grid.resolution + submap_grid.y0;
            point.z = 0.0;
            
            // check if there's conflict in other submap
            bool is_free = true;
            for(auto other_submap:submap_ids){
                if(GetPointIntensity(point,other_submap)>=kOccupyThreshold){
                    is_free = false;
                    break;
                }
            }
            if(is_free) return point;
        }
    }
}
   

void PathPlannerNode::AddSubmapGrid(const SubmapId& submap_id){
    // Clear the existing old data
    submap_grid_.erase(submap_id);

    // first fetch the submaptexture
    cartographer_ros_msgs::SubmapEntry& submap_entry = submap_[submap_id];
    auto fetched_textures = ::cartographer_ros::FetchSubmapTextures(submap_id, &submap_query_client_);
    const auto submap_texture = fetched_textures->textures.begin();
    
    // use fake map to do it (map only contain one element)
    std::map<SubmapId, SubmapSlice> fake_submap_slices;
    ::cartographer::io::SubmapSlice& submap_slice = fake_submap_slices[submap_id];
    
    submap_slice.pose = ToRigid3d(submap_entry.pose);
    submap_slice.metadata_version = submap_entry.submap_version;
    
    // push fetched texture to slice and draw the texture
    submap_slice.version = fetched_textures->version;
    submap_slice.width = submap_texture->width;
    submap_slice.height = submap_texture->height;
    submap_slice.slice_pose = submap_texture->slice_pose;
    submap_slice.resolution = submap_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(submap_texture->pixels.intensity,
                                                           submap_texture->pixels.alpha,
                                                           submap_texture->width, submap_texture->height,
                                                           &submap_slice.cairo_data);
    
    
    // Paint the texture
    auto painted_slices = cartographer::io::PaintSubmapSlices(fake_submap_slices, kOccupyGridResolution);
    
    // Convert painted surface into occupied grid
    auto& submap_grid = submap_grid_[submap_id];
    const Eigen::Array2f& origin = painted_slices.origin;
    cairo_surface_t* surface = painted_slices.surface.get();
    const int width = cairo_image_surface_get_width(surface);
    const int height = cairo_image_surface_get_height(surface);
    
    submap_grid.submap_id = submap_id;
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
    std::cout<<"Successfully Add Submap "<<submap_id<<" into submap_grid_"<<std::endl;
}
    

// Reconnect two submaps if necessary
bool PathPlannerNode::ReconnectSubmaps(const SubmapId& start_id, const SubmapId& end_id){
    Path path = PlanPathRRT(start_id,end_id);
    if(path.empty()) return false;
    double path_length = LengthOfPath(path);
    SubmapConnectState start_submap_connection_state (start_id,end_id,path_length);
    SubmapConnectState end_submap_connection_state (end_id,end_id,path_length);
    start_submap_connection_state.path = path;
    std::reverse(path.begin(),path.end());
    end_submap_connection_state.path = std::move(path);
    road_map_[start_id][end_id] = std::move(start_submap_connection_state);
    road_map_[end_id][start_id] = std::move(end_submap_connection_state);
    return true;
}

    
void PathPlannerNode::AddRoadMapEntry(const SubmapId& submap_id){
    std::cout<<"Try to add sumbap into road map "<<submap_id<<std::endl;
    auto& submap_entry = submap_[submap_id];
    if(submap_entry.submap_version!=kFinishVersion) return;
    
    // Find neigbor submaps that are potential for adding
    std::vector<SubmapId> near_submaps = CloseSubmaps(submap_entry.pose.position, parameters_.distance_threshold_for_adding);
    SubmapId next_submap {submap_id.trajectory_id,submap_id.submap_index+1};
    SubmapId last_submap {submap_id.trajectory_id,submap_id.submap_index+1};

    near_submaps.push_back(std::move(next_submap));
    near_submaps.push_back(std::move(last_submap));

    for(auto& near_submap:near_submaps){
        // Check near_submap exists and is not submap itself
        if(submap_.count(near_submap)==0) continue;
        if(near_submap==submap_id) continue;
        
        // Connect two submpas
        Path path = PlanPathRRT(submap_id,near_submap);
        
        if(path.empty()){
            std::cout<<"Warning!! Fail to connect "<<submap_id<<" , "<<near_submap<<std::endl;
            continue;
        }
        
        // Add path into road_map
        double path_length = LengthOfPath(path);
        
        SubmapConnectState submap_connect_state (submap_id,
                                                 near_submap,
                                                 path_length);
        SubmapConnectState other_submap_connect_state (near_submap,
                                                       submap_id,
                                                       path_length);
        
        submap_connect_state.path = path;
        std::reverse(path.begin(),path.end());
        other_submap_connect_state.path = std::move(path);
        road_map_[submap_id][near_submap] = std::move(submap_connect_state);
        road_map_[near_submap][submap_id] = std::move(other_submap_connect_state);
    }
}
    
// update the road map every time received submaplist
void PathPlannerNode::UpdateRoadMap(const cartographer_ros_msgs::SubmapList::ConstPtr& msg){
    ::cartographer::common::MutexLocker locker(&mutex_);
    
    // check if submap has been changed
    for(auto& submap_entry:msg->submap){
        SubmapId submap_id {submap_entry.trajectory_id, submap_entry.submap_index};
        if(submap_entry.submap_version < kFinishVersion) continue;
        
        // add new finished submap into road_map_
        if(submap_.find(submap_id)==submap_.end()){
            submap_[submap_id] = submap_entry;
            submap_kdtree_.AddPointToKdTree(submap_entry.pose.position, submap_id.trajectory_id, submap_id.submap_index);
            AddSubmapGrid(submap_id);
            AddRoadMapEntry(submap_id);
        } else{
            auto& old_submap_entry = submap_[submap_id];
            double distance2 = Distance2BetweenPose(old_submap_entry.pose,submap_entry.pose);
            double rotation = RotationBetweenPose(old_submap_entry.pose,submap_entry.pose);
            if(rotation > parameters_.rotation_threshold_for_updating ||
               distance2 > parameters_.distance2_threshold_for_updating){
                submap_[submap_id] = submap_entry;
                AddSubmapGrid(submap_id);
                AddRoadMapEntry(submap_id);
            }
        }
    }
}
 
    
bool PathPlannerNode::ReconnectSubmapService(::cartographer_ros_msgs::ReconnectSubmaps::Request &req,
                                             ::cartographer_ros_msgs::ReconnectSubmaps::Response &res){
    SubmapId start_submap_id {req.start_trajectory_id, req.start_submap_index};
    SubmapId end_submap_id {req.end_trajectory_id, req.end_submap_index};
    ReconnectSubmaps(start_submap_id,end_submap_id);
    return true;
}
    
bool PathPlannerNode::QueryRoadmap(::cartographer_ros_msgs::RoadmapQuery::Request &req,
                                  ::cartographer_ros_msgs::RoadmapQuery::Response &res){
    SubmapId submap_id {req.trajectory_id, req.submap_index};
    const auto& pair = road_map_.find(submap_id);
    if(pair==road_map_.end()){
        std::cout<<"Submap "<<submap_id<<" do not exist!"<<std::endl;
        return true;
    }
    std::cout<<"Submap "<<submap_id<<" Connections: ";
    for(const auto& connection_state:pair->second){
        std::cout<<connection_state.first<<" ";
        res.connections.push_back(connection_state.first.trajectory_id);
        res.connections.push_back(connection_state.first.submap_index);
    }
    std::cout<<std::endl;
    return true;
}
    
bool PathPlannerNode::QueryConnection(cartographer_ros_msgs::ConnectionQuery::Request &req,
                                     cartographer_ros_msgs::ConnectionQuery::Response &res){
    SubmapId start_submap_id {req.start_trajectory_id, req.start_submap_index};
    SubmapId end_submap_id {req.end_trajectory_id, req.end_submap_index};
    const auto& pair = road_map_.find(start_submap_id);
    if(pair==road_map_.end()){
        std::cout<<"Submap "<<start_submap_id<<" do not exist!"<<std::endl;
        return true;
    }
    const auto& entry = pair->second.find(end_submap_id);
    if(entry==pair->second.end()){
        return true;
    }
    res.path = entry->second.path;
    return true;
}
    
bool PathPlannerNode::PlanPath(cartographer_ros_msgs::PathPlan::Request &req,
                              cartographer_ros_msgs::PathPlan::Response &res)  {
    geometry_msgs::Point start_point = req.start_point;
    geometry_msgs::Point end_point = req.end_point;
    res.path = PlanPathRRT(start_point,end_point);
    if(!res.path.empty()) AddDisplayPath(res.path);
    return true;
}
    
void PathPlannerNode::PublishPath(const ::ros::WallTimerEvent& timer_event){
    path_publisher_.publish(path_to_display_);
}
    
/*
Functions below are for test
*/

void PathPlannerNode::IsClickedPointFree(const geometry_msgs::PointStamped::ConstPtr& msg) const {
    //std::cout<<msg->point.x<<","<<msg->point.y<<":";
    std::cout<<IsFree(msg->point);
    std::cout<<std::endl;
}

void PathPlannerNode::NavigateToClickedPoint(const geometry_msgs::PointStamped::ConstPtr& msg) {
    std::cout<<"Received Goal:"<<msg->point.x<<","<<msg->point.y<<std::endl;
    geometry_msgs::Point departure;
    departure.x = 0.01;
    departure.y = 0.01;
    departure.z = 0.0;
    Path path = PlanPathRRT(departure,msg->point);
    if(path.empty()){
        std::cout<<"Fail to find a valid path!"<<std::endl;
        SubmapId submap_id = ClosestSubmap(msg->point);
        std::cout<<departure<<" :: "<<submap_id<<", "<<GetPointIntensity(msg->point,submap_id)<<std::endl;
    }
    AddDisplayPath(path);
    SubmapId submap_id = ClosestSubmap(msg->point);
    std::cout<<departure<<" :: "<<submap_id<<", "<<GetPointIntensity(msg->point,submap_id)<<std::endl;
}
    

} // namespace cartographer_ros_path_planner
} // namespace cartographer_ros
