#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot {

class PlannerCore {
 public:
  explicit PlannerCore(const rclcpp::Logger& logger);

  void set_goal(const geometry_msgs::msg::PoseStamped& goal);
  void update_odometry(const nav_msgs::msg::Odometry& odom);
  nav_msgs::msg::Path plan_path();
  bool has_goal() const { return has_goal_; }
  bool is_goal_reached() const;
  std::string get_target_frame() const;

 private:
  rclcpp::Logger logger_;
  
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Odometry current_odom_;
  bool has_goal_ = false;
  std::string target_frame_ = "sim_world";
  
  static constexpr double GOAL_TOLERANCE = 0.1;
  bool is_valid_quaternion(const geometry_msgs::msg::Quaternion& q) const;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
