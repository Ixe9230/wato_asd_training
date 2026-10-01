#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    //main entry point - given the current path + robot pose, returns the velocity command to publish
    // returns std::nullopt if there's no path or we're already at the goal
    std::optional<geometry_msgs::msg::Twist> computeVelocity(
      const nav_msgs::msg::Path& path,
      const nav_msgs::msg::Odometry& odom);

  private:
    rclcpp::Logger logger_;

    //tuning params
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    // find the point on the path roughly lookahead_distance_ away from the robot
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
      const nav_msgs::msg::Path& path, double robot_x, double robot_y);

    //distance between two points
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

    //quaternion -> yaw
    double extractYaw(const geometry_msgs::msg::Quaternion& quat);
};

}

#endif