// MIT License
//
// Copyright (c) 2021 Avery Girven
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

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav2_util/geometry_utils.hpp"
#include "std_msgs/msg/header.hpp"

namespace WayPoint_Publisher
{
class WayPoint_Publisher : public rclcpp::Node
{
public:
  explicit WayPoint_Publisher(rclcpp::NodeOptions options);

private:
  void waypoint_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);
  void startWaypointFollowing();
	void get_Waypoints();

	std::string filename;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose;
  using WaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
};
}  // namespace WayPoint Publisher

#endif  // WAYPOINT_PUBLISHER__WAYPOINT_PUBLISHER_HPP_
