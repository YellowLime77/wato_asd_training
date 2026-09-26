#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void initControlCore(double lookahead_distance, double goal_tolerance,
                         double linear_speed, double max_angular_speed);
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,
                                              const geometry_msgs::msg::Pose& robot_pose) const;

  private:
    geometry_msgs::msg::Point findLookaheadPoint(const nav_msgs::msg::Path& path,
                                                 const geometry_msgs::msg::Point& robot_position) const;
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& q) const;

    rclcpp::Logger logger_;

    double lookahead_distance_ = 1.5;
    double goal_tolerance_ = 0.3;
    double linear_speed_ = 0.8;
    double max_angular_speed_ = 1.0;
};

}

#endif
