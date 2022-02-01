# Waypoint Publisher

## Summary
This package contains a ROS2 node that interfaces with the 
Waypoint Follower in the nav2 stack that takes in a csv file 
in points described of by x, y, z, roll, pitch, yaw. it then hooks into 
the initial pose message to ensure the initial pose is set and its 
set in the map frame before it publishes the vector of poses to the 
WayPoint Follower.

## Topics 

## Publishers

  - nav2_msgs::action::FollowWaypoints
  - nav2_msgs::action::FollowWaypoints::Goal();


## Subscribers

  - /initialpose

## Params 

- this package can taken in one param which is a csv file of points you want the robot to follow in order
 
## Launch 

by default follow waypoints is set to false. so I exposed a launch arguement called follow_waypoints that you would tack onto the end  of your launch command.
for example to launch mammoth in waypoint mode you would type `ros2 launch mammoth_snowplow mammoth.launch.py follow_waypoints:=true`
and the node will listen for the initialpose to be published and then start navigating from there.

