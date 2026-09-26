#include <chrono>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  double lookahead_distance = this->declare_parameter("lookahead_distance", 1.5);
  double goal_tolerance = this->declare_parameter("goal_tolerance", 0.3);
  double linear_speed = this->declare_parameter("linear_speed", 0.8);
  double max_angular_speed = this->declare_parameter("max_angular_speed", 1.0);
  control_.initControlCore(lookahead_distance, goal_tolerance, linear_speed, max_angular_speed);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 10 Hz
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    // Planner is done with this goal (or can't find a path). Stop once, then stay quiet
    // so we don't fight with the teleop panel.
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    current_path_ = nullptr;
    return;
  }
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }
  cmd_vel_pub_->publish(control_.computeVelocity(*current_path_, robot_odom_->pose.pose));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
