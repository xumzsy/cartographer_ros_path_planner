# Cartographer ROS Path Planner
This package provides online 2D path planning based on cartographer submaps and does not rely on the global occupied grid. Main idea is to build up a global road map for submaps and use RRT* as local planner.

![Example image](cartographer_ros_path_planner/example.png)
## Getting Started
### Prerequisites
The package runs on Ubuntu 16.04 and ROS Kinetic.
### Installing
* Follow [cartographer_ros](https://github.com/googlecartographer/cartographer_ros) to download cartographer, cacartographer_ros and 2d demo.
* Move files in "src" to cartographer_ros/cartographer_ros/cartographer_ros
* Move files in "srv" to cartographer_ros/cartographer_ros_msgs/srv
* Move files in "launch" to cartographer_ros/cartographer_ros/launch
* Substitute cartographer_ros/cartographer_ros_msgs/CmakeLists.txt with the cmake file in srv
* Remake cartographer_ros

### Running the tests
* After launching 2d demo, open another terminal and launch the planner
```
roslaunch cartographer_ros cartographer_path_planner.launch
```
* Use the "clicked point" to play with it. The planner will plan a path between map origin to clicked point.
* In a few case, the path may go through the wall. This is mainly due to incorrect submaps created by cartographer.

## Methods
### Submap Road Map
Kd tree is used to store each submap's origin. For each submap, I try to connect it to nearby submaps in map, last submap and the next submap in time series. RRT* is used to do local planning and the result path is stored in a map with submap id as key.

### Global Path Plan
I decompose the global path plan into three parts: locally plan paths to connect start point and end point to nearest submaps. Use BFS to find the shortest path between these two submaps in road map. Finally combine the three paths.

## Parameters
### Hard Coded Parameters (const)
* kFinishVersion = 180. This is given by cartographer.
* kOccupyThreshold = 64. 0 is definitely free and 100 is definitely occupied.
* kOccupyGridResolution = 0.05 [m] Decided by cartographer configuration but hard coded here.
* kCloseSubmapRadius = 5.0 [m] Radius to find nearby submaps to given point
### ROS parameters in launch file
* max_rrt_node_num. Max num of node RRT will contain.
* step_to_check_reach_endpoint. Decide how often try to connect end point to RRT
* distance_threshold_for_adding. Distance threshold to decide whether try to connect two submaps
* distance_threshold_for_updating. If the position of the submap changes over this threshold, it will recalculate its road map
* rotation_threshold_for_updating. If the oretation (q.z) of the submap changes over this threshold, it will recalculate its road map
* probability_of_choose_endpoint. Probability to choose end point instead of random point as new point in RRT.
* rrt_grow_step and rrt_trim_radius. Parameters in RRT* algorithm.

## ROS API
* "/roadmap_query": Receive the SubmapId and return corresponding submaps connections
* "/connection_query": Receive two SubmapId and return the path in road map between them
* "/reconnect_submaps": Receive two SubmapId, discard existing path and try to reconnect them
* "/plan_path": Receive two geometry_msgs::Point and return a path connecting them
## C++ API
path_planner_node.h is well documented and provides useful information

## Future Work
### Improvement
* Add functions to smooth the path
* Add more RRT* tricks to accelerate convergence of RRT*

### Navigation
To work as navigation stack, continues real time localization and velocity controller are needed. Now cartographer supports pure localization as a new trajectory in map builder but has not provided ROS API.

Velocity controller transforms trajectory of way points to smooth velocity command which can be directly sent to robot's motion controller.

Since the submap are not perfect, dynamic navigation is desired. This however needs changes in cartographer and can not be implemented just in ROS.
