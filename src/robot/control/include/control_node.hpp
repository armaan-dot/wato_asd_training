#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr    path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;

  // State
  nav_msgs::msg::Path current_path_;
  double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};
  bool   path_received_{false};
  bool   odom_received_{false};

  // Pure Pursuit parameters
  static constexpr double LOOKAHEAD_DIST  = 1.0;   // metres
  static constexpr double LINEAR_SPEED    = 0.4;   // m/s
  static constexpr double MAX_ANGULAR     = 1.5;   // rad/s
  static constexpr double GOAL_TOLERANCE  = 0.4;   // metres — stop threshold
  static constexpr double CONTROL_HZ      = 10.0;  // timer rate

  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // Helpers
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
  double quaternionToYaw(double x, double y, double z, double w) const;
  double pointDistance(double x1, double y1, double x2, double y2) const;
};

#endif  // CONTROL_NODE_HPP_