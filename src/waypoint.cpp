// MIT License
//
// Copyright (c) 2021 Avery Girven & Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "waypoint/waypoint.hpp"

#include <fstream>
#include <iostream>

namespace WayPoint_Publisher
{
WayPoint_Publisher::WayPoint_Publisher(rclcpp::NodeOptions options)
    : Node("waypoint_publisher", options)
{
  filename = this->declare_parameter("filename", " ");
  follow_gps = this->declare_parameter("follow_gps", false);

  // follow waypoints if false, else follow gps points
  if (!follow_gps)
  {
    waypoint_follower_action_client =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            this, "/follow_waypoints");
    waypoint_follower_goal = nav2_msgs::action::FollowWaypoints::Goal();

    get_Waypoints();

    // hook into the initial pose topic
    initialpose = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&WayPoint_Publisher::waypoint_callback, this,
                  std::placeholders::_1));
  }
  else
  {
    gps = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gps/points", 10,
        std::bind(&WayPoint_Publisher::gps_callback, this,
                  std::placeholders::_1));
  }
}

void WayPoint_Publisher::get_Waypoints()
{
  tf2::Quaternion quaternion;
  std::ifstream my_waypoints(filename);
  geometry_msgs::msg::PoseStamped pose;
  int position;
  std::string line, cell;
  double roll, pitch, yaw;

  if (my_waypoints.is_open())
  {
    std::getline(my_waypoints, line);

    while (std::getline(my_waypoints, line))
    {
      std::stringstream streamline(line);
      position = 0;
      while (std::getline(streamline, cell, ','))
      {
        switch (position++)
        {
          case 0:
            pose.pose.position.x = std::stof(cell);  // get the x value
            break;
          case 1:
            pose.pose.position.y = std::stof(cell);  // get the y value
            break;
          case 2:
            pose.pose.position.z = std::stof(cell);  // get the z value
            break;
          case 3:
            roll = std::stof(cell);  // get the roll value
            break;
          case 4:
            pitch = std::stof(cell);  // get the pitch value
            break;
          case 5:
            yaw = std::stof(cell);  // get the yaw value
            break;
          case 6:
            quaternion.setRPY(roll, pitch, yaw);  // convert to quaterion
            pose.pose.orientation.x = quaternion.getX();
            pose.pose.orientation.y = quaternion.getY();
            pose.pose.orientation.z = quaternion.getZ();
            pose.pose.orientation.w = quaternion.getW();
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            waypoints.push_back(pose);
            break;
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "WayPoint Publisher could not open the waypoint file:(");
    return;
  }
  my_waypoints.close();
}

// This function... she needs some TLC
void WayPoint_Publisher::startWaypointFollowing()
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = waypoints;

  auto is_action_server_ready =
      waypoint_follower_action_client->wait_for_action_server(
          std::chrono::seconds(5));
  if (!is_action_server_ready)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "FollowWaypoints Action Server is not available:("
                 "Is the WayPoint Follower running?");
    return;
  }

  waypoint_follower_goal.poses = poses;

  RCLCPP_INFO(this->get_logger(), "Sending a path of %zu waypoints:)",
              waypoint_follower_goal.poses.size());
  for (auto waypoint : waypoint_follower_goal.poses)
  {
    RCLCPP_DEBUG(this->get_logger(), "\t(%lf, %lf)", waypoint.pose.position.x,
                 waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};

  auto future_goal_handle = waypoint_follower_action_client->async_send_goal(
      waypoint_follower_goal, send_goal_options);

  // Get the goal handle and save so that we can check on completion
  waypoint_follower_goal_handle = future_goal_handle.get();
  std::cout << waypoint_follower_goal_handle << std::endl;
  if (!waypoint_follower_goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server:(");
    return;
  }
}

void WayPoint_Publisher::waypoint_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  if (initialpose->header.frame_id != "map")
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Initial Pose was not set in the Map Frame:(");
    return;
  }

  // check to make sure we have waypoints to follow
  if (waypoints.empty())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Please Provide a set of waypoints to follow and try again!");
    return;
  }

  // if we get here, we are good to follow waypoints
  startWaypointFollowing();
}

void WayPoint_Publisher::gps_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // make sure the pose array isnt empty
  if (msg->poses.empty())
  {
    RCLCPP_ERROR(
        this->get_logger(),
        "Please provide a non empty ordered set of Gps points and try again!");
    return;
  }

  // populate the waypoint vector with gps points
  for (auto point : msg->poses)
  {
    geometry_msgs::msg::PoseStamped pointStamped;

    pointStamped.pose.set__position(point.position);
    pointStamped.pose.set__orientation(point.orientation);

    pointStamped.header.frame_id = "map";
    pointStamped.header.stamp = this->get_clock()->now();

    waypoints.push_back(pointStamped);
  }
  startWaypointFollowing();
}
}  // namespace WayPoint_Publisher
