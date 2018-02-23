/*
 Define some common basic functions
 */


#ifndef common_h
#define common_h

#include <iostream>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros {
namespace cartographer_ros_navigation{
  
// Return distance2 in XY plane
float Distance2BetweenPoint(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2){
    return (point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y);
}
    
float Distance2BetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2){
    return (pose1.position.x-pose2.position.x)*(pose1.position.x-pose2.position.x)+
    (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y)
    +(pose1.position.z-pose2.position.z)*(pose1.position.z-pose2.position.z);
}
    
geometry_msgs::Point operator+(const geometry_msgs::Point& a, const geometry_msgs::Point& b){
    geometry_msgs::Point sum;
    sum.x = a.x + b.x;
    sum.y = a.y + b.y;
    sum.z = a.z + b.z;
    return sum;
}

geometry_msgs::Point operator*(float a, const geometry_msgs::Point& b){
    geometry_msgs::Point product;
    product.x = a * b.x;
    product.y = a * b.y;
    product.z = a * b.z;
    return product;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point& point){
    os<<" "<<point.x<<","<<point.y<<","<<point.z<<" ";
    return os;
}

std::ostream& operator<<(std::ostream& os, Path& path){
    for(geometry_msgs::Point& point: path){
        os<<point<<std::endl;
    }
    return os;
}
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros

#endif /* common_h */
