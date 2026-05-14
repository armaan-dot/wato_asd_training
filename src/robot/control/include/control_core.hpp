#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace robot {

class ControlCore {
 public:
  explicit ControlCore(const rclcpp::Logger& logger);

  void set_path(const nav_msgs::msg::Path& path);
  void update_odometry(const nav_msgs::msg::Odometry& odom);
  geometry_msgs::msg::Twist compute_velocity();
  bool has_path() const { return has_path_; }

 private:
  rclcpp::Logger logger_;
  
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Odometry current_odom_;
  bool has_path_ = false;
  size_t current_waypoint_idx_ = 0;
  
  static constexpr double MAX_LINEAR_VEL = 0.5;
  static constexpr double MAX_ANGULAR_VEL = 1.0;
  static constexpr double WAYPOINT_TOLERANCE = 0.15;
  static constexpr double KP_LINEAR = 1.0;
  static constexpr double KP_ANGULAR = 2.0;
  
  double get_yaw_from_odometry() const;
  double normalize_angle(double angle) const;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
