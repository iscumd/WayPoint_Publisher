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

#ifndef WAYPOINT_PUBLISHER__WAYPOINT_PUBLISHER_HPP_
#define WAYPOINT_PUBLISHER__WAYPOINT_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace WayPoint_Publisher
{
class WayPoint_Publisher : public rclcpp::Node
{
  public:
  explicit WayPoint_Publisher(rclcpp::NodeOptions options);

  private:
  /**
   * @brief Takes in a csv file of waypoints 
   * and formats the points into a vector PoseStamped poses
   */
  void get_Waypoints();

  /**
   * @brief Main Callback function that is called
   * after the intial pose is set
   * @param initialpose 
   */
  void waypoint_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
          initialpose);

  /**
   * @brief Take in an order set of gps coordinates, formats 
   * them as a vector of pose Stamped and sends them off to waypoint follower
   * @param msg 
   */
  void gps_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  /**
   * @brief Takes in the vector of waypoints 
   * and publishes it to the WayPoint Follower node 
   * over the Action server 
   */
  void startWaypointFollowing();

  // class variables
  std::string filename{};
  bool follow_gps{};
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gps;

  // Action server
  using WaypointFollowerGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
      waypoint_follower_action_client;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle;
};
}  // namespace WayPoint_Publisher

#endif  // WAYPOINT_PUBLISHER__WAYPOINT_PUBLISHER_HPP_
