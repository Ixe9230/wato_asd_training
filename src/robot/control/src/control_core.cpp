#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger),
    lookahead_distance_(1.0),
    goal_tolerance_(0.3),
    linear_speed_(0.5) {}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

//quaternion -> yaw, same formula we used in map_memory_node
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

//walk the path, find the first point at least lookahead_distance_ away from the robot
std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, double robot_x, double robot_y) {
  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_x;
  robot_pos.y = robot_y;

  for (const auto& pose : path.poses) {
    double dist = computeDistance(robot_pos, pose.pose.position);
    if (dist >= lookahead_distance_) {
      return pose.pose.position;
    }
  }

  //nothing far enough away - fall back to the last point on the path (near the goal)
  if (!path.poses.empty()) {
    return path.poses.back().pose.position;
  }

  return std::nullopt;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path, const nav_msgs::msg::Odometry& odom) {

  if (path.poses.empty()) {
    return std::nullopt;
  }

  double robot_x = odom.pose.pose.position.x;
  double robot_y = odom.pose.pose.position.y;
  double robot_yaw = extractYaw(odom.pose.pose.orientation);

  // heck if we're basically at the final goal
  const auto& goal = path.poses.back().pose.position;
  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_x;
  robot_pos.y = robot_y;
  if (computeDistance(robot_pos, goal) < goal_tolerance_) {
    return std::nullopt;  // caller should publish a stop command
  }

  auto lookahead = findLookaheadPoint(path, robot_x, robot_y);
  if (!lookahead) {
    return std::nullopt;
  }

  //transform lookahead point into the robot's own frame
  double dx = lookahead->x - robot_x;
  double dy = lookahead->y - robot_y;

  double local_x = dx * std::cos(-robot_yaw) - dy * std::sin(-robot_yaw);
  double local_y = dx * std::sin(-robot_yaw) + dy * std::cos(-robot_yaw);

  double L = std::sqrt(local_x * local_x + local_y * local_y);
  if (L < 1e-3) {
    return std::nullopt;  // lookahead point is basically on top of the robot, avoid divide by zero
  }

  // pure pursuit curvature formula
  double curvature = 2.0 * local_y / (L * L);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = curvature * linear_speed_;

  return cmd_vel;
}

}