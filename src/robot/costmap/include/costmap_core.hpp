#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap(double resolution, int width, int height, double inflation_radius);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    nav_msgs::msg::OccupancyGrid getCostmap() const { return costmap_; }

  private:
    bool toGrid(double x, double y, int& gx, int& gy) const;
    void inflateObstacles(const std::vector<std::pair<int, int>>& obstacles);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid costmap_;
    double inflation_radius_ = 1.0;
};

}

#endif
