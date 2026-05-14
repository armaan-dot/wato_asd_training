#include <cmath>
#include <vector>
#include <algorithm>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// ── helpers ──────────────────────────────────────────────────────────────────

void CostmapNode::initCostmap(nav_msgs::msg::OccupancyGrid &grid)
{
  grid.header.frame_id = "robot";
  grid.info.resolution = RESOLUTION;
  grid.info.width      = GRID_SIZE;
  grid.info.height     = GRID_SIZE;

  // Origin: centre the grid around the robot
  grid.info.origin.position.x = -(GRID_SIZE * RESOLUTION) / 2.0;
  grid.info.origin.position.y = -(GRID_SIZE * RESOLUTION) / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.assign(GRID_SIZE * GRID_SIZE, 0);
}

void CostmapNode::markObstacle(nav_msgs::msg::OccupancyGrid &grid, int cx, int cy)
{
  if (cx < 0 || cx >= GRID_SIZE || cy < 0 || cy >= GRID_SIZE) return;
  grid.data[cy * GRID_SIZE + cx] = MAX_COST;
}

void CostmapNode::inflateObstacles(nav_msgs::msg::OccupancyGrid &grid)
{
  // Work on a copy so existing obstacles don't influence each other's inflation
  std::vector<int8_t> inflated = grid.data;

  int radius_cells = static_cast<int>(std::ceil(INFLATION_R / RESOLUTION));

  for (int y = 0; y < GRID_SIZE; ++y) {
    for (int x = 0; x < GRID_SIZE; ++x) {
      if (grid.data[y * GRID_SIZE + x] == MAX_COST) {
        // Inflate around this obstacle cell
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;

            double dist = std::hypot(dx * RESOLUTION, dy * RESOLUTION);
            if (dist > INFLATION_R) continue;

            int cost = static_cast<int>(MAX_COST * (1.0 - dist / INFLATION_R));
            int idx  = ny * GRID_SIZE + nx;
            if (cost > inflated[idx]) {
              inflated[idx] = static_cast<int8_t>(cost);
            }
          }
        }
      }
    }
  }
  grid.data = inflated;
}

void CostmapNode::publishCostmap(nav_msgs::msg::OccupancyGrid &grid)
{
  grid.header.stamp = this->get_clock()->now();
  costmap_pub_->publish(grid);
}

// ── main callback ─────────────────────────────────────────────────────────────

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  nav_msgs::msg::OccupancyGrid grid;
  initCostmap(grid);

  double angle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max || !std::isfinite(range)) continue;

    double wx = range * std::cos(angle);  // robot-frame x
    double wy = range * std::sin(angle);  // robot-frame y

    // Convert to grid cell (origin is at grid centre)
    int cx = static_cast<int>((wx - grid.info.origin.position.x) / RESOLUTION);
    int cy = static_cast<int>((wy - grid.info.origin.position.y) / RESOLUTION);

    markObstacle(grid, cx, cy);
  }

  inflateObstacles(grid);
  publishCostmap(grid);
}

// ── entry point ───────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}