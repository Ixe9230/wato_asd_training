#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // Initialize the constructs and their parameters
  //string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
//this runs every time we get a new scan
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.initializeCostmap(); //resets the grid before processing the scan
  RCLCPP_INFO(this->get_logger(), "Got scan with %zu beams", scan->ranges.size());

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) continue; // skip bad readings

    double angle = scan->angle_min + i * scan->angle_increment;
    costmap_.markObstacle(range, angle);
  }

  costmap_.inflateObstacles();
  publishCostmap();
}
/*
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
*/

// builds the OccupancyGrid msg from our costmap and publishes it
void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "robot/chassis/lidar"; // check this matches your actual lidar frame

  msg.info.resolution = costmap_.getResolution();
  msg.info.width = costmap_.getWidth();
  msg.info.height = costmap_.getHeight();
  msg.info.origin.position.x = costmap_.getOriginX();
  msg.info.origin.position.y = costmap_.getOriginY();
  msg.info.origin.orientation.w = 1.0;

  msg.data = costmap_.getGrid();

  costmap_pub_->publish(msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}