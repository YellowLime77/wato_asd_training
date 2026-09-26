#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initCostmap(double resolution, int width, int height, double inflation_radius) {
  inflation_radius_ = inflation_radius;

  costmap_.info.resolution = resolution;
  costmap_.info.width = width;
  costmap_.info.height = height;
  // Robot sits in the middle of the grid
  costmap_.info.origin.position.x = -width * resolution / 2.0;
  costmap_.info.origin.position.y = -height * resolution / 2.0;
  costmap_.info.origin.orientation.w = 1.0;
  costmap_.data.assign(width * height, -1);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  const double res = costmap_.info.resolution;
  const int width = costmap_.info.width;

  // Start every scan with an unknown map, only what the lidar sees gets filled in
  std::fill(costmap_.data.begin(), costmap_.data.end(), -1);

  std::vector<std::pair<int, int>> obstacles;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isnan(range) || range < scan->range_min) {
      continue;
    }

    // inf / max range means the beam didn't hit anything
    bool hit = range < scan->range_max;
    if (!hit) {
      range = scan->range_max;
    }

    double angle = scan->angle_min + i * scan->angle_increment;
    double c = std::cos(angle);
    double s = std::sin(angle);

    // Everything along the beam before the hit is free space
    int gx, gy;
    for (double r = 0.0; r < range; r += res / 2.0) {
      if (!toGrid(r * c, r * s, gx, gy)) {
        break;
      }
      costmap_.data[gy * width + gx] = 0;
    }

    if (hit && toGrid(range * c, range * s, gx, gy)) {
      obstacles.emplace_back(gx, gy);
    }
  }

  inflateObstacles(obstacles);
}

bool CostmapCore::toGrid(double x, double y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - costmap_.info.origin.position.x) / costmap_.info.resolution));
  gy = static_cast<int>(std::floor((y - costmap_.info.origin.position.y) / costmap_.info.resolution));
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(costmap_.info.width) && gy < static_cast<int>(costmap_.info.height);
}

void CostmapCore::inflateObstacles(const std::vector<std::pair<int, int>>& obstacles) {
  const int max_cost = 100;
  const int width = costmap_.info.width;
  const int height = costmap_.info.height;
  const double res = costmap_.info.resolution;
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / res));

  for (const auto& [ox, oy] : obstacles) {
    costmap_.data[oy * width + ox] = max_cost;
  }

  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int x = ox + dx;
        int y = oy + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }

        double dist = std::hypot(dx, dy) * res;
        if (dist >= inflation_radius_) {
          continue;
        }

        int8_t cost = static_cast<int8_t>(max_cost * (1.0 - dist / inflation_radius_));
        int8_t& cell = costmap_.data[y * width + x];
        if (cost > cell) {
          cell = cost;
        }
      }
    }
  }
}

}
