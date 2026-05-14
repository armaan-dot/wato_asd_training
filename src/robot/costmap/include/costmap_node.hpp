#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // Costmap parameters
  static constexpr double RESOLUTION    = 0.1;  // metres per cell
  static constexpr int    GRID_SIZE     = 200;  // cells (covers 20 m × 20 m)
  static constexpr double INFLATION_R   = 1.0;  // metres
  static constexpr int    MAX_COST      = 100;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void initCostmap(nav_msgs::msg::OccupancyGrid &grid);
  void markObstacle(nav_msgs::msg::OccupancyGrid &grid, int cx, int cy);
  void inflateObstacles(nav_msgs::msg::OccupancyGrid &grid);
  void publishCostmap(nav_msgs::msg::OccupancyGrid &grid);
};

#endif  // COSTMAP_NODE_HPP_