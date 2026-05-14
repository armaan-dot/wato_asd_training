#include "planner_core.hpp"
#include <cmath>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

bool PlannerCore::is_valid_quaternion(const geometry_msgs::msg::Quaternion& q) const {
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return norm > 0.1;
}

void PlannerCore::set_goal(const geometry_msgs::msg::PoseStamped& goal) {
  goal_ = goal;
  target_frame_ = goal.header.frame_id;
  
  // Validate and fix quaternion if needed
  if (!is_valid_quaternion(goal_.pose.orientation)) {
    RCLCPP_WARN(logger_, "Invalid quaternion received, using identity");
    goal_.pose.orientation.w = 1.0;
    goal_.pose.orientation.x = 0.0;
    goal_.pose.orientation.y = 0.0;
    goal_.pose.orientation.z = 0.0;
  }
  
  has_goal_ = true;
  RCLCPP_INFO(logger_, "Goal set in frame '%s': (%.2f, %.2f, %.2f)", 
              target_frame_.c_str(), goal_.pose.position.x, 
              goal_.pose.position.y, goal_.pose.position.z);
}

void PlannerCore::update_odometry(const nav_msgs::msg::Odometry& odom) {
  current_odom_ = odom;
}

bool PlannerCore::is_goal_reached() const {
  if (!has_goal_) return false;
  
  double dx = goal_.pose.position.x - current_odom_.pose.pose.position.x;
  double dy = goal_.pose.position.y - current_odom_.pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  return distance < GOAL_TOLERANCE;
}

std::string PlannerCore::get_target_frame() const {
  return target_frame_;
}

nav_msgs::msg::Path PlannerCore::plan_path() {
  nav_msgs::msg::Path path;
  path.header.stamp = rclcpp::Clock().now();
  
  if (!has_goal_) {
    path.header.frame_id = "sim_world";
    return path;
  }
  
  // Use odometry frame if available, otherwise use goal frame
  if (!current_odom_.header.frame_id.empty()) {
    path.header.frame_id = current_odom_.header.frame_id;
  } else {
    path.header.frame_id = goal_.header.frame_id;
  }
  
  // Create simple 2-point path: current position -> goal
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header = path.header;
  current_pose.pose = current_odom_.pose.pose;
  path.poses.push_back(current_pose);
  
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = path.header;
  goal_pose.pose = goal_.pose;
  path.poses.push_back(goal_pose);
  
  return path;
}

}  // namespace robot
