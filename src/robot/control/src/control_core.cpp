#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::initControlCore(double lookahead_distance, double goal_tolerance,
                                  double linear_speed, double max_angular_speed) {
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  max_angular_speed_ = max_angular_speed;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path,
                                                       const geometry_msgs::msg::Pose& robot_pose) const {
  geometry_msgs::msg::Twist cmd;  // all zeros = stop

  if (path.poses.empty()) {
    return cmd;
  }
  if (computeDistance(robot_pose.position, path.poses.back().pose.position) < goal_tolerance_) {
    return cmd;
  }

  geometry_msgs::msg::Point target = findLookaheadPoint(path, robot_pose.position);

  // Angle between where the robot is facing and the lookahead point
  double dx = target.x - robot_pose.position.x;
  double dy = target.y - robot_pose.position.y;
  double alpha = std::atan2(dy, dx) - extractYaw(robot_pose.orientation);
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));  // wrap to [-pi, pi]

  // If the point is way off to the side or behind us, turn on the spot first
  if (std::abs(alpha) > M_PI / 3.0) {
    cmd.angular.z = std::copysign(max_angular_speed_, alpha);
    return cmd;
  }

  // Pure pursuit: curvature of the arc that goes through the lookahead point
  double curvature = 2.0 * std::sin(alpha) / std::hypot(dx, dy);
  cmd.linear.x = linear_speed_;
  cmd.angular.z = std::clamp(linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

geometry_msgs::msg::Point ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path,
                                                          const geometry_msgs::msg::Point& robot_position) const {
  // Start searching from the closest point on the path so we never go backwards
  size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double d = computeDistance(path.poses[i].pose.position, robot_position);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }

  for (size_t i = closest; i < path.poses.size(); ++i) {
    if (computeDistance(path.poses[i].pose.position, robot_position) >= lookahead_distance_) {
      return path.poses[i].pose.position;
    }
  }
  // Near the end of the path, just aim for the goal
  return path.poses.back().pose.position;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}
