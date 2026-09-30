#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // sets up the global map's size/resolution, called once
    void initializeGlobalMap(double resolution, int width, int height, double origin_x, double origin_y);

    //takes a costmap + the robot's current position, merges it into the global map
    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw);

    //getter so map_memory_node.cpp can grab the current global map to publish
    const nav_msgs::msg::OccupancyGrid& getGlobalMap() const { return global_map_; }

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_;
};

}  

#endif  
