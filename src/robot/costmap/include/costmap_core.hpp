#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    nav_msgs::msg::OccupancyGrid convertLaserScanToCostmap(
        const sensor_msgs::msg::LaserScan& scan);

  private:
    rclcpp::Logger logger_;

    static constexpr double RESOLUTION = 0.1;
    static constexpr double GRID_WIDTH = 20.0;
    static constexpr double GRID_HEIGHT = 20.0;
    static constexpr double INFLATION_RADIUS = 1.0;
    static constexpr int8_t OCCUPIED_COST = 100;
    static constexpr int8_t FREE_COST = 0;
    static constexpr int8_t UNKNOWN_COST = -1;

    int grid_size_x_;
    int grid_size_y_;
    std::vector<int8_t> grid_;

    void initializeGrid();
    void markObstacles(const sensor_msgs::msg::LaserScan& scan);
    void inflateObstacles();
    void gridToWorld(int x, int y, double& world_x, double& world_y) const;
    void worldToGrid(double world_x, double world_y, int& x, int& y) const;
};

}

#endif