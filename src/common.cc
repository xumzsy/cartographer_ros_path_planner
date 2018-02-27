/*
 *
 *
 *
 */


#include "common.h"

namespace cartographer_ros {
namespace cartographer_ros_path_planner{

// Return distance2 in XY plane
double Distance2BetweenPoint(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2){
    return (point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y);
}
    
double Distance2BetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    return (pose1.position.x-pose2.position.x)*(pose1.position.x-pose2.position.x)+
    (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y)
    +(pose1.position.z-pose2.position.z)*(pose1.position.z-pose2.position.z);
}
    
} // namespace cartographer_ros_path_planner
} // namespace cartographer_ros
