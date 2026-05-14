#include "control_core.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::set_path(const nav_msgs::msg::Path& path) {
  path_ = path;
  has_path_ = (path_.poses.size() > 1);
  current_waypoint_idx_ = 0;
  
  // Validate path contains valid positions
  for (const auto& pose : path_.poses) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y)) {
      RCLCPP_WARN(logger_, "Path contains invalid (NaN/Inf) positions");
      has_path_ = false;
      return;
    }
  }
  
  if (has_path_) {
    RCLCPP_INFO(logger_, "Path set with %zu waypoints", path_.poses.size());
  }
}

void ControlCore::update_odometry(const nav_msgs::msg::Odometry& odom) {
  current_odom_ = odom;
}

double ControlCore::get_yaw_from_odometry() const {
  tf2::Quaternion q(
    current_odom_.pose.pose.orientation.x,
    current_odom_.pose.pose.orientation.y,
    current_odom_.pose.pose.orientation.z,
    current_odom_.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double ControlCore::normalize_angle(double angle) const {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

geometry_msgs::msg::Twist ControlCore::compute_velocity() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;

  if (!has_path_ || path_.poses.size() <= current_waypoint_idx_ + 1) {
    return cmd;
  }

  // Get target waypoint
  auto& target_pose = path_.poses[current_waypoint_idx_ + 1];
  double target_x = target_pose.pose.position.x;
  double target_y = target_pose.pose.position.y;

  // Validate target is finite
  if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
    RCLCPP_WARN(logger_, "Target waypoint contains invalid values");
    return cmd;
  }

  // Current position
  double curr_x = current_odom_.pose.pose.position.x;
  double curr_y = current_odom_.pose.pose.position.y;
  double curr_yaw = get_yaw_from_odometry();

  // Distance to target
  double dx = target_x - curr_x;
  double dy = target_y - curr_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Check if waypoint reached
  if (distance < WAYPOINT_TOLERANCE) {
    current_waypoint_idx_++;
    if (current_waypoint_idx_ >= path_.poses.size() - 1) {
      return cmd;
    }
    return compute_velocity();
  }

  // Calculate desired heading
  double desired_yaw = std::atan2(dy, dx);
  double yaw_error = normalize_angle(desired_yaw - curr_yaw);

  // Proportional control
  double linear_vel = std::min(KP_LINEAR * distance, MAX_LINEAR_VEL);
  double angular_vel = std::clamp(KP_ANGULAR * yaw_error, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  // Only move forward if reasonably aligned
  if (std::abs(yaw_error) > M_PI / 2) {
    linear_vel = 0.0;
  }

  cmd.linear.x = linear_vel;
  cmd.angular.z = angular_vel;

  return cmd;
}

}  // namespace robot
