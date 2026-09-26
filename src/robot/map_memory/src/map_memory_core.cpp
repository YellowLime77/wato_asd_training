#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution, int width, int height, double origin_x, double origin_y,
                            const std::string& frame_id) {
  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width * height, -1);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  const auto& map_info = global_map_.info;
  const auto& cm_info = costmap.info;
  const int cm_width = cm_info.width;
  const int cm_height = cm_info.height;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);

  // Go through every cell in the global map and look up which costmap cell lands on it.
  // Doing it this way (instead of pushing costmap cells into the map) means no holes
  // show up when the costmap is rotated.
  for (unsigned int y = 0; y < map_info.height; ++y) {
    for (unsigned int x = 0; x < map_info.width; ++x) {
      double wx = map_info.origin.position.x + (x + 0.5) * map_info.resolution;
      double wy = map_info.origin.position.y + (y + 0.5) * map_info.resolution;

      // World -> robot frame
      double dx = wx - robot_x;
      double dy = wy - robot_y;
      double rx = c * dx + s * dy;
      double ry = -s * dx + c * dy;

      int cx = static_cast<int>(std::floor((rx - cm_info.origin.position.x) / cm_info.resolution));
      int cy = static_cast<int>(std::floor((ry - cm_info.origin.position.y) / cm_info.resolution));
      if (cx < 0 || cy < 0 || cx >= cm_width || cy >= cm_height) {
        continue;
      }

      // New data wins, but unknown cells keep whatever we had before
      int8_t cost = costmap.data[cy * cm_width + cx];
      if (cost >= 0) {
        global_map_.data[y * map_info.width + x] = cost;
      }
    }
  }
}

}
