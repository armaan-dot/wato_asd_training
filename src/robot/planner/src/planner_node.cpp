#include <cmath>
#include <array>
#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner")
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(TIMER_PERIOD_MS)),
    std::bind(&PlannerNode::timerCallback, this));
}

// ── helpers ───────────────────────────────────────────────────────────────────

CellIndex PlannerNode::worldToCell(const nav_msgs::msg::OccupancyGrid &map,
                                   double wx, double wy) const
{
  int cx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
  int cy = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
  return {cx, cy};
}

bool PlannerNode::cellValid(const nav_msgs::msg::OccupancyGrid &map,
                             const CellIndex &c) const
{
  if (c.x < 0 || c.x >= static_cast<int>(map.info.width))  return false;
  if (c.y < 0 || c.y >= static_cast<int>(map.info.height)) return false;
  int8_t cost = map.data[c.y * map.info.width + c.x];
  return cost >= 0 && cost < OBSTACLE_THRESH;
}

double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b) const
{
  // Euclidean distance in cell space
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool PlannerNode::goalReached() const
{
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < GOAL_TOLERANCE;
}

// ── A* ────────────────────────────────────────────────────────────────────────

std::optional<std::vector<CellIndex>> PlannerNode::astar(
  const nav_msgs::msg::OccupancyGrid &map,
  CellIndex start, CellIndex goal_cell) const
{
  using MinHeap = std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>>;

  std::unordered_map<CellIndex, CellIndex,   CellIndexHash> came_from;
  std::unordered_map<CellIndex, double,      CellIndexHash> g_score;
  std::unordered_map<CellIndex, bool,        CellIndexHash> closed;

  g_score[start] = 0.0;

  MinHeap open;
  open.emplace(start, heuristic(start, goal_cell));

  // 8-connected neighbours
  const std::array<std::pair<int,int>, 8> dirs{{
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
  }};

  while (!open.empty()) {
    AStarNode current = open.top(); open.pop();
    CellIndex idx = current.index;

    if (idx == goal_cell) {
      // Reconstruct path
      std::vector<CellIndex> path;
      CellIndex cur = goal_cell;
      while (cur != start) {
        path.push_back(cur);
        cur = came_from[cur];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    if (closed[idx]) continue;
    closed[idx] = true;

    for (auto [dx, dy] : dirs) {
      CellIndex nb{idx.x + dx, idx.y + dy};
      if (!cellValid(map, nb) || closed[nb]) continue;

      double step_cost = std::hypot(dx, dy);
      // Add a small penalty for high-cost cells (inflation zone)
      double cell_cost = map.data[nb.y * map.info.width + nb.x] / 100.0;
      double tentative_g = g_score[idx] + step_cost + cell_cost;

      if (g_score.find(nb) == g_score.end() || tentative_g < g_score[nb]) {
        g_score[nb]   = tentative_g;
        came_from[nb] = idx;
        open.emplace(nb, tentative_g + heuristic(nb, goal_cell));
      }
    }
  }
  return std::nullopt;  // No path found
}

// ── plan & publish ────────────────────────────────────────────────────────────

void PlannerNode::planPath()
{
  if (!map_received_ || !goal_received_) return;

  CellIndex start_cell = worldToCell(current_map_, robot_x_, robot_y_);
  CellIndex goal_cell  = worldToCell(current_map_, goal_.point.x, goal_.point.y);

  auto result = astar(current_map_, start_cell, goal_cell);
  if (!result) {
    RCLCPP_WARN(this->get_logger(), "A*: no path found!");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp    = this->get_clock()->now();
  path_msg.header.frame_id = "sim_world";

  for (const auto &cell : *result) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x =
      current_map_.info.origin.position.x + (cell.x + 0.5) * current_map_.info.resolution;
    ps.pose.position.y =
      current_map_.info.origin.position.y + (cell.y + 0.5) * current_map_.info.resolution;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }

  path_pub_->publish(path_msg);
  RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints.", path_msg.poses.size());
}

// ── callbacks ─────────────────────────────────────────────────────────────────

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_  = *msg;
  map_received_ = true;
  if (state_ == State::NAVIGATING) planPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_          = *msg;
  goal_received_ = true;
  state_         = State::NAVIGATING;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}

void PlannerNode::timerCallback()
{
  if (state_ == State::NAVIGATING) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;

      // Publish empty path to stop the controller
      nav_msgs::msg::Path empty;
      empty.header.stamp    = this->get_clock()->now();
      empty.header.frame_id = "odom";
      path_pub_->publish(empty);
    }
  }
}

// ── entry point ───────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}