#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <queue>
#include <unordered_map>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ── A* support structures ──────────────────────────────────────────────────────

struct CellIndex {
  int x, y;
  CellIndex(int xx = 0, int yy = 0) : x(xx), y(yy) {}
  bool operator==(const CellIndex &o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex &o) const { return !(*this == o); }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 16);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  bool operator>(const AStarNode &o) const { return f_score > o.f_score; }
};

// ── Planner node ───────────────────────────────────────────────────────────────

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  enum class State { WAITING_FOR_GOAL, NAVIGATING };
  State state_{State::WAITING_FOR_GOAL};

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                 path_pub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;

  // Data
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  double robot_x_{0.0}, robot_y_{0.0};
  bool map_received_{false};
  bool goal_received_{false};

  // Parameters
  static constexpr double GOAL_TOLERANCE   = 0.5;   // metres
  static constexpr int    OBSTACLE_THRESH  = 50;    // cells with cost ≥ this are blocked
  static constexpr double TIMER_PERIOD_MS  = 500.0;

  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // A* helpers
  void planPath();
  bool goalReached() const;
  std::optional<std::vector<CellIndex>> astar(
    const nav_msgs::msg::OccupancyGrid &map,
    CellIndex start, CellIndex goal_cell) const;
  CellIndex worldToCell(const nav_msgs::msg::OccupancyGrid &map,
                        double wx, double wy) const;
  bool cellValid(const nav_msgs::msg::OccupancyGrid &map,
                 const CellIndex &c) const;
  double heuristic(const CellIndex &a, const CellIndex &b) const;
};

#endif  // PLANNER_NODE_HPP_