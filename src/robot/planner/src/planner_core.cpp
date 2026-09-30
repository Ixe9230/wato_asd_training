#include "planner_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <queue>
#include <unordered_set>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

//meters -> grid cell
CellIndex PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy) const {
  int gx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int gy = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(gx, gy);
}

//grid cell -> meters (center of cell)
void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell, double& wx, double& wy) const {
  wx = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
  wy = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
}

// checks if a cell is in bounds and not blocked
bool PlannerCore::isValid(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& cell) const {
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }

  int idx = cell.y * map.info.width + cell.x;
  int8_t cost = map.data[idx];

  // treat unknown (-1) as walkable since otherwise huge unexplored areas
  //become impossible to path through - just block confirmed high-cost cells
    if (cost > 50) return false;
  return true;
}

//just euclidean distance, using it as the heuristic
double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                            double start_x, double start_y,
                            double goal_x, double goal_y,
                            nav_msgs::msg::Path& path) {
  CellIndex start = worldToGrid(map, start_x, start_y);
  CellIndex goal = worldToGrid(map, goal_x, goal_y);

  if (!isValid(map, start) || !isValid(map, goal)) {
    RCLCPP_WARN(logger_, "Start or goal cell is not walkable");
    return false;
  }

  //priority queue, lowest f_score comes out first
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  //best cost found so far to reach each cell
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  // so we can trace the path back once we hit the goal
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  //cells we're done checking, don't need to look at again
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  //checking all 8 directions around a cell, not just up/down/left/right
  std::vector<CellIndex> directions = {
    CellIndex(1, 0), CellIndex(-1, 0), CellIndex(0, 1), CellIndex(0, -1),
    CellIndex(1, 1), CellIndex(1, -1), CellIndex(-1, 1), CellIndex(-1, -1)
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      // found it, now trace back through came_from to get the actual path
      std::vector<CellIndex> cell_path;
      CellIndex c = current;
      while (!(c == start)) {
        cell_path.push_back(c);
        c = came_from[c];
      }
      cell_path.push_back(start);
      std::reverse(cell_path.begin(), cell_path.end()); // was backwards, start -> goal now

      //turn the grid cells into an actual Path message
      for (const auto& cell : cell_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "sim_world";
        double wx, wy;
        gridToWorld(map, cell, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

      return true;
    }

    if (closed_set.count(current)) continue; // already did this one
    closed_set.insert(current);

    for (const auto& dir : directions) {
      CellIndex neighbor(current.x + dir.x, current.y + dir.y);

      if (!isValid(map, neighbor)) continue;
      if (closed_set.count(neighbor)) continue;

      // diagonals cost more (sqrt2) since theyre longer than a straight step
      double move_cost = (dir.x != 0 && dir.y != 0) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + move_cost;

      //only update if this is a cheaper way to reach this neighbor
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f));
      }
    }
  }

  //ran out of cells to check and never hit the goal, so no path exists
  RCLCPP_WARN(logger_, "No path found to goal");
  return false;
}

}