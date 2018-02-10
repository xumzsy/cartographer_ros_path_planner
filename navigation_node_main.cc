#include "ros/ros.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "navigation_node.h"

int main(int argc, char** argv){
  ::ros::init(argc, argv, "cartographer_navigation_node");
  cartographer_ros_navigation::Node node;
  ::ros::spin();

}
