#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  grid_size_x_ = static_cast<int>(GRID_WIDTH / RESOLUTION);
  grid_size_y_ = static_cast<int>(GRID_HEIGHT / RESOLUTION);
  initializeGrid();
}

void CostmapCore::initializeGrid() {
  grid_.assign(grid_size_x_ * grid_size_y_, FREE_COST);
}

nav_msgs::msg::OccupancyGrid CostmapCore::convertLaserScanToCostmap(
    const sensor_msgs::msg::LaserScan& scan) {
  initializeGrid();
  markObstacles(scan);
  inflateObstacles();

  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan.header;
  grid.header.frame_id = scan.header.frame_id;

  grid.info.resolution = RESOLUTION;
  grid.info.width = grid_size_x_;
  grid.info.height = grid_size_y_;

  grid.info.origin.position.x = -GRID_WIDTH / 2.0;
  grid.info.origin.position.y = -GRID_HEIGHT / 2.0;
  grid.info.origin.position.z = 0.0;

  grid.info.origin.orientation.w = 1.0;

  grid.data = grid_;

  return grid;
}

void CostmapCore::markObstacles(const sensor_msgs::msg::LaserScan& scan) {
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];

    if (range < scan.range_min || range > scan.range_max) {
      continue;
    }

    double angle = scan.angle_min + i * scan.angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);

    if (grid_x >= 0 && grid_x < grid_size_x_ && grid_y >= 0 && grid_y < grid_size_y_) {
      grid_[grid_y * grid_size_x_ + grid_x] = OCCUPIED_COST;
    }
  }
}

void CostmapCore::inflateObstacles() {
  std::vector<int8_t> inflated_grid = grid_;

  for (int y = 0; y < grid_size_y_; ++y) {
    for (int x = 0; x < grid_size_x_; ++x) {
      if (grid_[y * grid_size_x_ + x] == OCCUPIED_COST) {
        for (int iy = 0; iy < grid_size_y_; ++iy) {
          for (int ix = 0; ix < grid_size_x_; ++ix) {
            double dx = (ix - x) * RESOLUTION;
            double dy = (iy - y) * RESOLUTION;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance <= INFLATION_RADIUS && distance > 0) {
              int8_t cost = static_cast<int8_t>(
                  OCCUPIED_COST * (1.0 - distance / INFLATION_RADIUS));
              inflated_grid[iy * grid_size_x_ + ix] =
                  std::max(inflated_grid[iy * grid_size_x_ + ix], cost);
            }
          }
        }
      }
    }
  }

  grid_ = inflated_grid;
}

void CostmapCore::worldToGrid(double world_x, double world_y, int& x, int& y) const {
  x = static_cast<int>((world_x + GRID_WIDTH / 2.0) / RESOLUTION);
  y = static_cast<int>((world_y + GRID_HEIGHT / 2.0) / RESOLUTION);
}

void CostmapCore::gridToWorld(int x, int y, double& world_x, double& world_y) const {
  world_x = x * RESOLUTION - GRID_WIDTH / 2.0;
  world_y = y * RESOLUTION - GRID_HEIGHT / 2.0;
}

}