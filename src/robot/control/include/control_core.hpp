#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cmath>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,
                                              const geometry_msgs::msg::Pose& robot_pose);

  private:
    rclcpp::Logger logger_;

    static constexpr double LOOKAHEAD_DISTANCE = 1.0;
    static constexpr double LINEAR_SPEED = 0.5;
    static constexpr double GOAL_TOLERANCE = 0.1;

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& robot_pose);
    double computeDistance(const geometry_msgs::msg::Point& a,
                          const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;
    double normalizeAngle(double angle) const;
};

}

#endif
