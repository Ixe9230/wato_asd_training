#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  //set up the global map once - bigger than a single costmap so it can
  // cover more of the environment as the robot explores
  map_memory_.initializeGlobalMap(0.1, 600, 600, -30.0, -30.0);

  //check every 1 second whether we should fuse + publish
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

//just stash the latest costmap, the timer decides when to actually use it
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

// track position, and figure out yaw from the quaternion
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  //quaternion -> yaw (rotation around vertical axis)
  auto q = msg->pose.pose.orientation;
  current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double distance = std::sqrt(
    std::pow(current_x_ - last_x_, 2) + std::pow(current_y_ - last_y_, 2));

  if (distance >= distance_threshold_) {
    last_x_ = current_x_;
    last_y_ = current_y_;
    should_update_map_ = true;
  }
}

//runs once a second, only does work if we've moved far enough AND have new data
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_received_) {
    map_memory_.fuseCostmap(latest_costmap_, current_x_, current_y_, current_yaw_);

    auto global_map = map_memory_.getGlobalMap();
    global_map.header.stamp = this->get_clock()->now();
    global_map.header.frame_id = "sim_world"; // check this matches your actual world frame

    map_pub_->publish(global_map);

    should_update_map_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}