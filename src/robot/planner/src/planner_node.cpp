#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", rclcpp::QoS(10).transient_local(),
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  //checks every 500ms if we reached the goal or need to replan
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

//new map came in - if we're actively navigating, replan since the map changed
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

//got a new goal - switch states and plan a path to it right away
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

//just keep track of where the robot currently is
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

//runs every 500ms - checks if we're done, or replans if we're not making progress
void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      planPath();
    }
  }
}

//close enough counts as reached, don't need to be exact
bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

//actually runs A* and publishes whatever it finds
void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or goal");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world"; // matches the frame_id used for /map

  bool found = planner_.planPath(
    current_map_,
    robot_pose_.position.x, robot_pose_.position.y,
    goal_.point.x, goal_.point.y,
    path);

  if (found) {
    RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints", path.poses.size());
    path_pub_->publish(path);
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid path found");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}