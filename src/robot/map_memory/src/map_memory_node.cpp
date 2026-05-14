#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory")
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(TIMER_PERIOD_S),
    std::bind(&MapMemoryNode::timerCallback, this));

  initGlobalMap();
}

// ── helpers ───────────────────────────────────────────────────────────────────

void MapMemoryNode::initGlobalMap()
{
  global_map_.header.frame_id    = "odom";
  global_map_.info.resolution    = MAP_RESOLUTION;
  global_map_.info.width         = MAP_SIZE;
  global_map_.info.height        = MAP_SIZE;
  global_map_.info.origin.position.x = -(MAP_SIZE * MAP_RESOLUTION) / 2.0;
  global_map_.info.origin.position.y = -(MAP_SIZE * MAP_RESOLUTION) / 2.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(MAP_SIZE * MAP_SIZE, -1);  // -1 = unknown
  map_initialized_ = true;
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) const
{
  // Standard quaternion → yaw (rotation around Z)
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void MapMemoryNode::integrateCostmap()
{
  // The costmap is in robot frame; we need to transform each cell into the
  // global (odom) frame and write it into global_map_.

  const auto &cm   = latest_costmap_;
  double cm_res    = cm.info.resolution;
  int    cm_w      = cm.info.width;
  int    cm_h      = cm.info.height;

  double cos_yaw = std::cos(robot_yaw_);
  double sin_yaw = std::sin(robot_yaw_);

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      int8_t cell_val = cm.data[cy * cm_w + cx];
      if (cell_val < 0) continue;  // unknown — skip

      // Costmap cell centre in robot frame
      double lx = cm.info.origin.position.x + (cx + 0.5) * cm_res;
      double ly = cm.info.origin.position.y + (cy + 0.5) * cm_res;

      // Rotate + translate into global (odom) frame
      double gx = robot_x_ + cos_yaw * lx - sin_yaw * ly;
      double gy = robot_y_ + sin_yaw * lx + cos_yaw * ly;

      // Convert to global map indices
      int mx = static_cast<int>((gx - global_map_.info.origin.position.x) / MAP_RESOLUTION);
      int my = static_cast<int>((gy - global_map_.info.origin.position.y) / MAP_RESOLUTION);

      if (mx < 0 || mx >= MAP_SIZE || my < 0 || my >= MAP_SIZE) continue;

      int idx = my * MAP_SIZE + mx;
      // Overwrite with new data (new data wins; keep "unknown" if no reading)
      if (cell_val >= 0) {
        global_map_.data[idx] = cell_val;
      }
    }
  }
}

// ── callbacks ─────────────────────────────────────────────────────────────────

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_    = *msg;
  costmap_received_  = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  auto &q  = msg->pose.pose.orientation;
  robot_yaw_ = quaternionToYaw(q.x, q.y, q.z, q.w);

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  if (std::hypot(dx, dy) >= MOVE_THRESHOLD) {
    should_update_ = true;
  }
}

void MapMemoryNode::timerCallback()
{
  if (!costmap_received_ || !map_initialized_) return;

  // Always publish so the planner always has something to work with,
  // but only fuse when the robot has moved far enough.
  if (should_update_) {
    integrateCostmap();
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_ = false;
    RCLCPP_INFO(this->get_logger(), "Map updated at (%.2f, %.2f)", robot_x_, robot_y_);
  }

  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

// ── entry point ───────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}