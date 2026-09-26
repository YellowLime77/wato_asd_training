#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  double resolution = this->declare_parameter("resolution", 0.1);
  int width = this->declare_parameter("width", 400);
  int height = this->declare_parameter("height", 400);
  double origin_x = this->declare_parameter("origin_x", -20.0);
  double origin_y = this->declare_parameter("origin_y", -20.0);
  std::string frame_id = this->declare_parameter("frame_id", std::string("sim_world"));
  distance_threshold_ = this->declare_parameter("distance_threshold", 1.5);
  int update_period_ms = this->declare_parameter("update_period_ms", 1000);
  map_memory_.initMap(resolution, width, height, origin_x, origin_y, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(update_period_ms), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
  // Save the pose now rather than when the timer fires, otherwise the costmap
  // gets placed wherever the robot has moved to since the scan
  costmap_odom_ = latest_odom_;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = msg;
}

void MapMemoryNode::updateMap() {
  if (latest_costmap_ && costmap_odom_) {
    const auto& pose = costmap_odom_->pose.pose;
    double dist = std::hypot(pose.position.x - last_x_, pose.position.y - last_y_);

    // Always take the first costmap so the planner has something to work with right away
    if (first_update_ || dist >= distance_threshold_) {
      const auto& q = pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      map_memory_.integrateCostmap(*latest_costmap_, pose.position.x, pose.position.y, yaw);
      last_x_ = pose.position.x;
      last_y_ = pose.position.y;
      first_update_ = false;
    }
  }

  // Publish every tick (even with no changes) so anything that starts late still gets a map
  nav_msgs::msg::OccupancyGrid map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
