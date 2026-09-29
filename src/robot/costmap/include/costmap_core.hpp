#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

// Holds the grild and logic for building a costmap from lidar data
class CostmapCore {
  public:
    //Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    //Resets cells back to 0 before a new scan comes in
    void initializeCostmap();

    //Takes a lidar reading and marks the cell is occupied 
    void markObstacle(double range, double angle);

    //Spreads cost around occupied cells
    void inflateObstacles();

    //Getters so costmap_node.cpp can read this data 
    const std::vector<int8_t>& getGrid() const {return grid_;}
    int getWidth() const {return width_;}
    int getHeight() const {return height_;}
    double getResolution() const {return resolution_;}
    double getOriginX() const{return origin_x_;}
    double getOriginY() const{return origin_y_;}

  private:
    rclcpp::Logger logger_;

    //Setting up the grid!
    double resolution_; //Meters per cell
    int width_; //Grid width in cells
    int height_; //Grid height in cells 
    double origin_x_; 
    double origin_y_;

    //The grid
    std::vector<int8_t> grid_;

    //How far obstacle is from the danger zone
    double inflation_radius_;
    int8_t max_cost_; //Set the cells corresponding to detected obstacle positions to a high cost

    //Turns (x, y) into grid cell (gx, gy), if it is out of bounds it will be false
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;

    //Turns (gx, gy) into one index for the flat grid_ array
    int toIndex(int gx, int gy) const{return gy*width_ + gx;}

};

}  

#endif  