# WayPoint_Publisher
This package contains a ROS2 node that interfaces with the 
Waypoint Follower in the nav2 stack that takes in a csv file 
in points described of by x,y,z,roll,pitch,yaw. it then hooks into 
the initial pose message to ensure the initial pose is set and its 
set in the map frame before it publishes the vector of poses to the 
WayPoint Follower.

