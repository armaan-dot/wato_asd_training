#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                                          const geometry_msgs::msg::Pose& start,
                                          const geometry_msgs::msg::Pose& goal) {
  nav_msgs::msg::Path path;
  path.header = map.header;

  int start_x, start_y, goal_x, goal_y;
  worldToGrid(start.position.x, start.position.y, start_x, start_y, map);
  worldToGrid(goal.position.x, goal.position.y, goal_x, goal_y, map);

  if (!isValid(CellIndex(start_x, start_y), map) || !isValid(CellIndex(goal_x, goal_y), map)) {
    return path;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  CellIndex start_idx(start_x, start_y);
  CellIndex goal_idx(goal_x, goal_y);

  g_score[start_idx] = 0;
  double h = heuristic(start_idx, goal_idx);
  open_set.push(AStarNode(start_idx, h));

  int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int costs[] = {DIAGONAL_COST, STRAIGHT_COST, DIAGONAL_COST, STRAIGHT_COST,
                 STRAIGHT_COST, DIAGONAL_COST, STRAIGHT_COST, DIAGONAL_COST};

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal_idx) {
      CellIndex cell = goal_idx;
      while (cell != start_idx) {
        double world_x, world_y;
        gridToWorld(cell.x, cell.y, world_x, world_y, map);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = map.header;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1.0;

        path.poses.insert(path.poses.begin(), pose);
        cell = came_from[cell];
      }

      double world_x, world_y;
      gridToWorld(start_idx.x, start_idx.y, world_x, world_y, map);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = map.header;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;
      path.poses.insert(path.poses.begin(), pose);

      return path;
    }

    if (closed_set.count(current.index)) continue;
    closed_set.insert(current.index);

    for (int i = 0; i < 8; ++i) {
      CellIndex neighbor(current.index.x + dx[i], current.index.y + dy[i]);

      if (!isValid(neighbor, map) || closed_set.count(neighbor)) {
        continue;
      }

      double tentative_g = g_score[current.index] + costs[i];

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;

        double h = heuristic(neighbor, goal_idx);
        double f = tentative_g + h;
        open_set.push(AStarNode(neighbor, f));
      }
    }
  }

  return path;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy) * 10.0;
}

bool PlannerCore::isValid(const CellIndex& index, const nav_msgs::msg::OccupancyGrid& map) const {
  if (index.x < 0 || index.x >= static_cast<int>(map.info.width) || index.y < 0 ||
      index.y >= static_cast<int>(map.info.height)) {
    return false;
  }

  int grid_index = index.y * map.info.width + index.x;
  if (grid_index >= static_cast<int>(map.data.size())) {
    return false;
  }

  int8_t cost = map.data[grid_index];
  return cost >= 0 && cost < OBSTACLE_THRESHOLD;
}

void PlannerCore::worldToGrid(double x, double y, int& grid_x, int& grid_y,
                              const nav_msgs::msg::OccupancyGrid& map) const {
  grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
}

void PlannerCore::gridToWorld(int grid_x, int grid_y, double& x, double& y,
                              const nav_msgs::msg::OccupancyGrid& map) const {
  x = grid_x * map.info.resolution + map.info.origin.position.x;
  y = grid_y * map.info.resolution + map.info.origin.position.y;
}

}
