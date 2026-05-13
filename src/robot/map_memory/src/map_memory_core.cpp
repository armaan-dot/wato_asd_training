#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger), map_initialized_(false) {
  initializeGlobalMap(
      static_cast<int>(GRID_WIDTH / RESOLUTION),
      static_cast<int>(GRID_HEIGHT / RESOLUTION), RESOLUTION);
}

void MapMemoryCore::initializeGlobalMap(int width, int height, double resolution) {
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;

  global_map_.info.origin.position.x = -GRID_WIDTH / 2.0;
  global_map_.info.origin.position.y = -GRID_HEIGHT / 2.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(width * height, -1);
  global_map_.header.frame_id = "map";

  map_initialized_ = true;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

void MapMemoryCore::integrateLocalMap(
    const nav_msgs::msg::OccupancyGrid& local_map,
    const geometry_msgs::msg::Pose& robot_pose) {
  double robot_x = robot_pose.position.x;
  double robot_y = robot_pose.position.y;
  double robot_yaw = extractYaw(robot_pose.orientation);

  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  for (size_t i = 0; i < local_map.data.size(); ++i) {
    if (local_map.data[i] < 0) continue;

    int local_x = i % local_map.info.width;
    int local_y = i / local_map.info.width;

    double local_world_x =
        local_x * local_map.info.resolution + local_map.info.origin.position.x;
    double local_world_y =
        local_y * local_map.info.resolution + local_map.info.origin.position.y;

    double global_x = robot_x + cos_yaw * local_world_x - sin_yaw * local_world_y;
    double global_y = robot_y + sin_yaw * local_world_x + cos_yaw * local_world_y;

    int global_grid_x = static_cast<int>(
        (global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
    int global_grid_y = static_cast<int>(
        (global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

    if (global_grid_x >= 0 && global_grid_x < static_cast<int>(global_map_.info.width) &&
        global_grid_y >= 0 && global_grid_y < static_cast<int>(global_map_.info.height)) {
      int index = global_grid_y * global_map_.info.width + global_grid_x;
      global_map_.data[index] = std::max(global_map_.data[index], local_map.data[i]);
    }
  }

  global_map_.header.stamp = local_map.header.stamp;
}

double MapMemoryCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}
