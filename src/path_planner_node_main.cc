/*
 * Author: Mengze Xu
 */

#include "ros/ros.h"
#include "path_planner_node.h"

int main(int argc, char** argv){
  ::ros::init(argc, argv, "cartographer_navigation_node");
  ::ros::start();
  cartographer_ros::cartographer_ros_navigation::NavigationNode node;
  ::ros::spin();
  ::ros::shutdown();
  return 0;
}
