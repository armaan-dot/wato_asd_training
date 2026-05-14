#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_pub_;
  rclcpp::TimerBase::SharedPtr                                   timer_;

  // State
  nav_msgs::msg::OccupancyGrid global_map_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool costmap_received_{false};
  bool map_initialized_{false};
  bool should_update_{false};

  double last_update_x_{0.0};
  double last_update_y_{0.0};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};

  // Parameters
  static constexpr double MAP_RESOLUTION      = 0.1;   // metres per cell
  static constexpr int    MAP_SIZE            = 400;   // 40 m × 40 m global map
  static constexpr double MOVE_THRESHOLD      = 1.5;   // metres before fusing
  static constexpr double TIMER_PERIOD_S      = 1.0;

  // Callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Helpers
  void initGlobalMap();
  void integrateCostmap();
  double quaternionToYaw(double x, double y, double z, double w) const;
};

#endif  // MAP_MEMORY_NODE_HPP_