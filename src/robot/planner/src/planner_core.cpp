#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void PlannerCore::initPlanner(int lethal_cost, double cost_weight) {
  lethal_cost_ = lethal_cost;
  cost_weight_ = cost_weight;
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Point& start,
                           const geometry_msgs::msg::Point& goal, nav_msgs::msg::Path& path) {
  const int width = map.info.width;
  const int height = map.info.height;
  const double res = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  auto toCell = [&](const geometry_msgs::msg::Point& p) {
    return CellIndex(static_cast<int>(std::floor((p.x - origin_x) / res)),
                     static_cast<int>(std::floor((p.y - origin_y) / res)));
  };
  auto inBounds = [&](const CellIndex& c) {
    return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height;
  };
  auto toIndex = [&](const CellIndex& c) { return c.y * width + c.x; };
  // Unknown cells (-1) are treated as free
  auto costAt = [&](const CellIndex& c) { return std::max<int>(0, map.data[toIndex(c)]); };

  CellIndex start_cell = toCell(start);
  CellIndex goal_cell = toCell(goal);
  if (!inBounds(start_cell) || !inBounds(goal_cell)) {
    RCLCPP_WARN(logger_, "Start or goal is outside the map");
    return false;
  }
  if (costAt(goal_cell) >= lethal_cost_) {
    RCLCPP_WARN(logger_, "Goal is too close to an obstacle");
    return false;
  }

  auto heuristic = [&](const CellIndex& c) {
    return std::hypot(c.x - goal_cell.x, c.y - goal_cell.y);
  };

  std::vector<double> g_score(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(width * height, -1);
  std::vector<bool> closed(width * height, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  g_score[toIndex(start_cell)] = 0.0;
  open_set.emplace(start_cell, heuristic(start_cell));

  bool found = false;
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal_cell) {
      found = true;
      break;
    }
    if (closed[toIndex(current)]) {
      continue;
    }
    closed[toIndex(current)] = true;

    int current_cost = costAt(current);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        CellIndex next(current.x + dx, current.y + dy);
        if (!inBounds(next) || closed[toIndex(next)]) {
          continue;
        }

        // Can't go into lethal cells. The exception is if we're already inside one
        // (robot got too close to something), then we're allowed to move out of it.
        int cost = costAt(next);
        if (cost >= lethal_cost_ && cost >= current_cost) {
          continue;
        }

        // Moving through higher cost cells is more expensive so the path stays away from obstacles
        double step = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
        double tentative_g = g_score[toIndex(current)] + step * (1.0 + cost_weight_ * cost / 100.0);
        if (tentative_g < g_score[toIndex(next)]) {
          g_score[toIndex(next)] = tentative_g;
          came_from[toIndex(next)] = toIndex(current);
          open_set.emplace(next, tentative_g + heuristic(next));
        }
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* couldn't find a path to the goal");
    return false;
  }

  // Walk back from the goal to get the path
  std::vector<int> cells;
  for (int i = toIndex(goal_cell); i != -1; i = came_from[i]) {
    cells.push_back(i);
  }
  std::reverse(cells.begin(), cells.end());

  path.poses.clear();
  for (int i : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = origin_x + (i % width + 0.5) * res;
    pose.pose.position.y = origin_y + (i / width + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // End exactly on the goal instead of the center of its cell
  path.poses.back().pose.position.x = goal.x;
  path.poses.back().pose.position.y = goal.y;

  return true;
}

}
