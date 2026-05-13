#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal);

  private:
    rclcpp::Logger logger_;

    static constexpr int OBSTACLE_THRESHOLD = 50;
    static constexpr int DIAGONAL_COST = 14;
    static constexpr int STRAIGHT_COST = 10;

    double heuristic(const CellIndex& a, const CellIndex& b) const;
    bool isValid(const CellIndex& index, const nav_msgs::msg::OccupancyGrid& map) const;
    void worldToGrid(double x, double y, int& grid_x, int& grid_y,
                     const nav_msgs::msg::OccupancyGrid& map) const;
    void gridToWorld(int grid_x, int grid_y, double& x, double& y,
                     const nav_msgs::msg::OccupancyGrid& map) const;
};

}

#endif
