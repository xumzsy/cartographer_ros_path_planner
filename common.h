/*
 Define some common basic functions
 */


#ifndef common_h
#define common_h

#include <iostream>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros {
namespace cartographer_ros_navigation{
using Path = std::vector<geometry_msgs::Point>;

// Return distance2 in XY plane
double Distance2BetweenPoint(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2);
    
double Distance2BetweenPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    
} // namespace cartographer_ros_navigation
} // namespace cartographer_ros

#endif /* common_h */
