#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  int lethal_cost = this->declare_parameter("lethal_cost", 50);
  double cost_weight = this->declare_parameter("cost_weight", 5.0);
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter("goal_timeout", 120.0);
  planner_.initPlanner(lethal_cost, cost_weight);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    stop();
  } else if ((this->now() - goal_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Took too long to reach the goal, giving up");
    stop();
  }
}

bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or odometry!");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_->header.frame_id;

  // If there's no path we publish an empty one so the robot stops instead of
  // following an old path into something
  planner_.planPath(*map_, robot_pose_.position, goal_.point, path);
  path_pub_->publish(path);
}

void PlannerNode::stop() {
  state_ = State::WAITING_FOR_GOAL;

  // Empty path tells the controller to stop
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_ ? map_->header.frame_id : "sim_world";
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
