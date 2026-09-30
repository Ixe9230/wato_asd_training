#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    //runs whenever a new costmap comes in, just stores it for later
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    //runs whenever odometry updates, tracks how far the robot has moved
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //runs on a timer, actually does the fuse + publish if conditions are met
    void updateMap();

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // latest costmap we've received, and whether it's new since last fuse
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;

    //robot position tracking, for the "moved far enough" check
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    bool should_update_map_ = false;

    const double distance_threshold_ = 1.5; //meters
};

#endif