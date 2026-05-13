#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    void integrateLocalMap(
        const nav_msgs::msg::OccupancyGrid& local_map,
        const geometry_msgs::msg::Pose& robot_pose);
    void initializeGlobalMap(int width, int height, double resolution);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool map_initialized_;

    static constexpr double GRID_WIDTH = 100.0;
    static constexpr double GRID_HEIGHT = 100.0;
    static constexpr double RESOLUTION = 0.1;

    void ensureMapCoverage(double x, double y);
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;
};

}

#endif
