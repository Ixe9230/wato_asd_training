#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <unordered_map>

namespace robot
{

//a single grid cell, used to identify positions during the search
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

//lets us use CellIndex as a key in an unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// one entry in the priority queue: a cell + its total estimated cost
struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

//tells the priority queue to always pop the LOWEST f_score first
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // runs A* on the given map from start to goal (all in world coordinates, meters)
    //returns true if a path was found, and fills `path` with the result
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double start_x, double start_y,
                  double goal_x, double goal_y,
                  nav_msgs::msg::Path& path);

  private:
    rclcpp::Logger logger_;

    //world coords (meters) -> grid cell
    CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy) const;

    // grid cell -> world coords (meters), center of the cell
    void gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell, double& wx, double& wy) const;

    //is this cell walkable? (in bounds + not occupied)
    bool isValid(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const;

    //straight-line distance heuristic
    double heuristic(const CellIndex& a, const CellIndex& b) const;
};

}

#endif