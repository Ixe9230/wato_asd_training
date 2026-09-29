#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

//Setting default values for the grid
CostmapCore::CostmapCore(const rclcpp::Logger& logger) 
    : logger_(logger),
    resolution_(0.1), //10cm per cell
    width_(300), //300 cells = 30m since resolution is 0.1
    height_(300),
    inflation_radius_(1.0), //1m buffer around obstacles
    max_cost_(100)

{
    //Want the robot to be roughly in the middle of the grid, not the corner
    origin_x_ = -(width_ * resolution_) / 2.0;
    origin_y_ = -(height_ * resolution_) / 2.0;

    initializeCostmap();
}

//reset everything to 0 before processing a new scan
void CostmapCore::initializeCostmap() {
    grid_.assign(width_ * height_, 0);
}

//convert a real world point to grid cell indexes
bool CostmapCore::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

// take one lidar point (range + angle) and mark it as occupied on the grid
void CostmapCore::markObstacle(double range, double angle) {
  // convert from polar to x/y using the formulas from the doc
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  int gx, gy;
  if (worldToGrid(x, y, gx, gy)) {
    grid_[toIndex(gx, gy)] = max_cost_;
  }
}

//spreads cost around each obstacle so the robot keeps some distance
void CostmapCore::inflateObstacles() {
  int cell_radius = static_cast<int>(inflation_radius_ / resolution_);

  // grab all the occupied cells first so we dont end up inflating
  // cells we just inflated in this same loop
  std::vector<int> occupied_indices;
  for (int i = 0; i < width_ * height_; ++i) {
    if (grid_[i] == max_cost_) {
      occupied_indices.push_back(i);
    }
  }

  for (int idx : occupied_indices) {
    int ox = idx % width_;
    int oy = idx / width_;

    //check every cell in a square around this obstacle
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        int nx = ox + dx;
        int ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
        if (distance > inflation_radius_) continue; // too far, skip

        //formula from the assignment doc, closer = higher cost
        int8_t cost = static_cast<int8_t>(
          max_cost_ * (1.0 - distance / inflation_radius_));

        int n_idx = toIndex(nx, ny);
        //only overwrite if this cost is actually higher
        if (cost > grid_[n_idx]) {
          grid_[n_idx] = cost;
        }
      }
    }
  }
}



}