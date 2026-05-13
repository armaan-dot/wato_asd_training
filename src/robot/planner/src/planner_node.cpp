#include <cmath>
#include "planner_node.hpp"

PlannerNode::PlannerNode()
    : Node("planner"), planner_(robot::PlannerCore(this->get_logger())),
      state_(State::WAITING_FOR_GOAL), goal_received_(false) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&PlannerNode::goalPointCallback, this, std::placeholders::_1));

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&PlannerNode::goalPoseCallback, this, std::placeholders::_1));

  move_base_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10,
      std::bind(&PlannerNode::goalPoseCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_.header = msg->header;
  goal_.pose.position.x = msg->point.x;
  goal_.pose.position.y = msg->point.y;
  goal_.pose.position.z = 0.0;
  goal_.pose.orientation.w = 1.0;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.pose.position.x - robot_pose_.position.x;
  double dy = goal_.pose.position.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance < 0.5;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    return;
  }

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = goal_.pose.position.x;
  goal_pose.position.y = goal_.pose.position.y;
  goal_pose.position.z = goal_.pose.position.z;
  goal_pose.orientation = goal_.pose.orientation;

  auto path = planner_.planPath(current_map_, robot_pose_, goal_pose);

  path_pub_->publish(path);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
