#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

//set up the global map once - bigger than a single costmap since it
// needs to cover the whole area the robot might explore
void MapMemoryCore::initializeGlobalMap(double resolution, int width, int height, double origin_x, double origin_y) {
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;

  // -1 = unknown, matches ROS's OccupancyGrid convention
  global_map_.data.assign(width * height, -1);
}

//take the local costmap (centered on robot) and copy it into the global map
//using the robot's position + yaw to figure out where each cell actually is
void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw) {
  int local_w = costmap.info.width;
  int local_h = costmap.info.height;
  double local_res = costmap.info.resolution;
  double local_origin_x = costmap.info.origin.position.x;
  double local_origin_y = costmap.info.origin.position.y;

  int global_w = global_map_.info.width;
  int global_h = global_map_.info.height;
  double global_res = global_map_.info.resolution;
  double global_origin_x = global_map_.info.origin.position.x;
  double global_origin_y = global_map_.info.origin.position.y;

  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  for (int ly = 0; ly < local_h; ++ly) {
    for (int lx = 0; lx < local_w; ++lx) {
      int local_idx = ly * local_w + lx;
      int8_t value = costmap.data[local_idx];

      if (value < 0) continue; //unknown cell in local costmap, skip it

      // local cell -> local costmap's own coordinate frame (meters, robot-centered)
      double local_px = local_origin_x + (lx + 0.5) * local_res;
      double local_py = local_origin_y + (ly + 0.5) * local_res;

      //rotate + translate into the global/world frame using robot's pose
      double world_x = robot_x + (local_px * cos_yaw - local_py * sin_yaw);
      double world_y = robot_y + (local_px * sin_yaw + local_py * cos_yaw);

      //world coords -> global map cell indices
      int gx = static_cast<int>((world_x - global_origin_x) / global_res);
      int gy = static_cast<int>((world_y - global_origin_y) / global_res);

      if (gx < 0 || gx >= global_w || gy < 0 || gy >= global_h) continue; // out of global map bounds

      int global_idx = gy * global_w + gx;
      global_map_.data[global_idx] = value; //new data overwrites old, per the assignment doc
    }
  }
}

} 
