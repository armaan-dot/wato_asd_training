#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path,
                                                       const geometry_msgs::msg::Pose& robot_pose) {
  geometry_msgs::msg::Twist cmd_vel;

  if (path.poses.empty()) {
    return cmd_vel;
  }

  auto lookahead_point = findLookaheadPoint(path, robot_pose);
  if (!lookahead_point) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  double target_x = lookahead_point->pose.position.x;
  double target_y = lookahead_point->pose.position.y;

  double dx = target_x - robot_pose.position.x;
  double dy = target_y - robot_pose.position.y;
  double distance_to_target = std::sqrt(dx * dx + dy * dy);

  if (distance_to_target < GOAL_TOLERANCE) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  double target_heading = std::atan2(dy, dx);
  double robot_yaw = extractYaw(robot_pose.orientation);

  double angle_error = normalizeAngle(target_heading - robot_yaw);

  double curvature = (2.0 * std::sin(angle_error)) / LOOKAHEAD_DISTANCE;
  double angular_velocity = LINEAR_SPEED * curvature;

  cmd_vel.linear.x = LINEAR_SPEED;
  cmd_vel.angular.z = angular_velocity;

  return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& robot_pose) {
  double min_distance = std::numeric_limits<double>::infinity();
  int closest_idx = -1;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = computeDistance(path.poses[i].pose.position, robot_pose.position);
    if (dist < min_distance) {
      min_distance = dist;
      closest_idx = i;
    }
  }

  if (closest_idx < 0) {
    return std::nullopt;
  }

  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    double dist = computeDistance(path.poses[i].pose.position, robot_pose.position);
    if (dist >= LOOKAHEAD_DISTANCE) {
      return path.poses[i];
    }
  }

  if (path.poses.size() > 0) {
    return path.poses.back();
  }

  return std::nullopt;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlCore::normalizeAngle(double angle) const {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

}
