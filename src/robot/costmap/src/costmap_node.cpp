#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  double resolution = this->declare_parameter("resolution", 0.1);
  int width = this->declare_parameter("width", 300);
  int height = this->declare_parameter("height", 300);
  double inflation_radius = this->declare_parameter("inflation_radius", 2.0);
  costmap_.initCostmap(resolution, width, height, inflation_radius);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateCostmap(msg);

  nav_msgs::msg::OccupancyGrid costmap = costmap_.getCostmap();
  // Costmap is centered on the lidar, so it uses the same frame and stamp as the scan
  costmap.header = msg->header;
  costmap_pub_->publish(costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
