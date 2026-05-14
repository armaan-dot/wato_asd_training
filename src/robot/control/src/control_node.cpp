#include <cmath>
#include <algorithm>
#include "control_node.hpp"

ControlNode::ControlNode() : Node("control")
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / CONTROL_HZ)),
    std::bind(&ControlNode::controlLoop, this));
}

// ── helpers ───────────────────────────────────────────────────────────────────

double ControlNode::quaternionToYaw(double x, double y, double z, double w) const
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double ControlNode::pointDistance(double x1, double y1, double x2, double y2) const
{
  return std::hypot(x2 - x1, y2 - y1);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const
{
  if (current_path_.poses.empty()) return std::nullopt;

  // Walk the path; return the first waypoint that is at least LOOKAHEAD_DIST away.
  // If every remaining point is closer (e.g. near the end), return the last point.
  for (const auto &pose : current_path_.poses) {
    double dist = pointDistance(robot_x_, robot_y_,
                                pose.pose.position.x, pose.pose.position.y);
    if (dist >= LOOKAHEAD_DIST) return pose;
  }
  return current_path_.poses.back();
}

// ── control loop (Pure Pursuit) ───────────────────────────────────────────────

void ControlNode::controlLoop()
{
  geometry_msgs::msg::Twist cmd;

  if (!path_received_ || !odom_received_ || current_path_.poses.empty()) {
    cmd_vel_pub_->publish(cmd);  // publish zero velocity
    return;
  }

  // Check if we have reached the final goal
  const auto &final_pose = current_path_.poses.back().pose;
  double dist_to_goal = pointDistance(robot_x_, robot_y_,
                                      final_pose.position.x, final_pose.position.y);
  if (dist_to_goal < GOAL_TOLERANCE) {
    RCLCPP_INFO(this->get_logger(), "Reached end of path. Stopping.");
    cmd_vel_pub_->publish(cmd);  // zero
    return;
  }

  // Find lookahead point
  auto lookahead = findLookaheadPoint();
  if (!lookahead) {
    cmd_vel_pub_->publish(cmd);
    return;
  }

  double lx = lookahead->pose.position.x;
  double ly = lookahead->pose.position.y;

  // Transform lookahead point into the robot's local frame
  double dx = lx - robot_x_;
  double dy = ly - robot_y_;

  // Angle from robot's heading to the lookahead point
  double angle_to_target = std::atan2(dy, dx);
  double heading_error   = angle_to_target - robot_yaw_;

  // Normalise to [-π, π]
  while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  // Pure Pursuit: curvature κ = 2·sin(α) / L
  double L = std::hypot(dx, dy);
  double curvature = 2.0 * std::sin(heading_error) / std::max(L, 0.001);

  cmd.linear.x  = LINEAR_SPEED;
  cmd.angular.z = std::clamp(LINEAR_SPEED * curvature, -MAX_ANGULAR, MAX_ANGULAR);

  cmd_vel_pub_->publish(cmd);
}

// ── callbacks ─────────────────────────────────────────────────────────────────

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_  = *msg;
  path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  auto &q  = msg->pose.pose.orientation;
  robot_yaw_   = quaternionToYaw(q.x, q.y, q.z, q.w);
  odom_received_ = true;
}

// ── entry point ───────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}