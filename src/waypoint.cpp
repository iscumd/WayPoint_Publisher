#include "waypoint/waypoint.hpp"
#include <fstream>
#include <iostream>

namespace WayPoint_Publisher 
{
WayPoint_Publisher::WayPoint_Publisher(rclcpp::NodeOptions options): Node("waypoint_publisher", options)
{
  filename = this->declare_parameter("filename", " ");
  waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this,
    "FollowWaypoints");
  waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

  get_Waypoints();

  // hook into the initial pose topic
  initialpose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&WayPoint_Publisher::waypoint_callback, this, std::placeholders::_1));
}

/* 
Takes in a csv file of waypoints 
and formats the points into a vector PoseStamped poses
*/
void WayPoint_Publisher::get_Waypoints()
{
  tf2::Quaternion quaternion;
  std::ifstream my_waypoints(filename);
  geometry_msgs::msg::PoseStamped pose;
  int position;
  std::string line, cell;
  double roll, pitch, yaw;

  if(my_waypoints.is_open()) 
  {
    std::getline(my_waypoints,line);

    while(std::getline(my_waypoints,line))
    {
      std::stringstream streamline(line);
      position = 0;
      while(std::getline(streamline, cell, ','))
      {
      switch(position++) {
        case 0:
          pose.pose.position.x = std::stof(cell);
          break;
        case 1:
          pose.pose.position.y = std::stof(cell);
          break;
        case 2:
          pose.pose.position.z = std::stof(cell);
          break;
        case 3:
          roll = std::stof(cell);
          break;
        case 4:
          pitch = std::stof(cell);
          break;
        case 5:
          yaw = std::stof(cell);
          break;
        case 6:
          quaternion.setRPY(roll,pitch,yaw);
          pose.pose.orientation.x = quaternion.getX();
          pose.pose.orientation.y = quaternion.getY();
          pose.pose.orientation.z = quaternion.getZ();
          pose.pose.orientation.w = quaternion.getW();
          pose.header.frame_id = "map";
          pose.header.stamp = this->get_clock()->now();
          waypoints_.push_back(pose);
          break;
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "WayPoint Publisher could not open the waypoint file:(");
    return;
  }
  my_waypoints.close();
}

/* 
Takes in the vector of waypoints 
and publishes it to the WayPoint Follower node 
over the Action server 
*/
void WayPoint_Publisher::startWaypointFollowing()
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = waypoints_;
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "FollowWaypoints Action Server is not available:("
      "Is the WayPoint Follower running?");
    return;
  }
  
  waypoint_follower_goal_.poses = poses;

  RCLCPP_INFO(
    this->get_logger(), "Sending a path of %zu waypoints:)",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [](auto) {};  
  
  auto future_goal_handle =
    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);

  // Get the goal handle and save so that we can check on completion
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  std::cout << waypoint_follower_goal_handle_ << std::endl;
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server:(");
    return;
  }
}

/*
Main Callback function that is called
after the intial pose is set
*/
void WayPoint_Publisher::waypoint_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  if(initialpose->header.frame_id != "map") {
    RCLCPP_ERROR(this->get_logger(), "Initial Pose was not set in the Map Frame:(");
    return;
  }
  startWaypointFollowing();
}

} // end of Waypoint_Publisher namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<WayPoint_Publisher::WayPoint_Publisher>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
