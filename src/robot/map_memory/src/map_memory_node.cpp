#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())),
      last_update_x_(0.0), last_update_y_(0.0), costmap_updated_(false) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));

  auto initial_map = map_memory_.getGlobalMap();
  map_pub_->publish(initial_map);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (costmap_updated_) {
    map_memory_.integrateLocalMap(latest_costmap_, msg->pose.pose);
    costmap_updated_ = false;
  }
}

void MapMemoryNode::timerCallback() {
  auto map = map_memory_.getGlobalMap();
  map_pub_->publish(map);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
