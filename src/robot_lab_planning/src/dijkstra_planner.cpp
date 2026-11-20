#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robot_lab_planning/dijkstra_planner.hpp"

namespace robot_lab_planning
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Cells at or above lethal_cost are blocked; the default 253 also blocks
  // the inscribed band, where the footprint would touch an obstacle.
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_cost", rclcpp::ParameterValue(253));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_factor", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_smoother", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".smoother_id", rclcpp::ParameterValue(std::string("simple_smoother")));
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".lethal_cost", lethal_cost_);
  node_->get_parameter(name_ + ".cost_factor", cost_factor_);
  node_->get_parameter(name_ + ".use_smoother", use_smoother_);
  node_->get_parameter(name_ + ".smoother_id", smoother_id_);
  lethal_cost_ = std::clamp(lethal_cost_, 1, 255);

  if (use_smoother_) {
    smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(
      node_, "smooth_path");
  }
}

void DijkstraPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type DijkstraPlanner",
    name_.c_str());
  smooth_client_.reset();
}

void DijkstraPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type DijkstraPlanner",
    name_.c_str());
  // A missing smoother only costs the smoothing; plans are still served.
  if (smooth_client_ && !smooth_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_WARN(
      node_->get_logger(), "%s: smooth_path is not available; plans stay unsmoothed",
      name_.c_str());
  }
}

void DijkstraPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type DijkstraPlanner",
    name_.c_str());
}

std::vector<unsigned int> DijkstraPlanner::search(
  const unsigned char * costs, unsigned int size_x, unsigned int size_y,
  unsigned int start, unsigned int goal, bool allow_unknown,
  unsigned char lethal_cost, double cost_factor)
{
  const unsigned int cells = size_x * size_y;
  auto passable = [&](unsigned int index) {
      const unsigned char cost = costs[index];
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown;
      }
      return cost < lethal_cost;
    };
  if (start >= cells || goal >= cells || !passable(goal)) {
    return {};
  }

  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> distance(cells, kInf);
  std::vector<unsigned int> parent(cells, cells);
  std::vector<bool> closed(cells, false);
  using Entry = std::pair<double, unsigned int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

  // The step into a cell costs its length, raised by the cell's cost so
  // the path keeps away from obstacles where it can.
  auto step_cost = [&](unsigned int index, double length) {
      const unsigned char cost = costs[index] == nav2_costmap_2d::NO_INFORMATION ?
        0 : costs[index];
      return length * (1.0 + cost_factor * cost / 252.0);
    };

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const double length[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

  // The start cell may lie in the inflation band; it is expanded anyway.
  distance[start] = 0.0;
  open.emplace(0.0, start);
  while (!open.empty()) {
    const auto [d, current] = open.top();
    open.pop();
    if (closed[current]) {
      continue;
    }
    closed[current] = true;
    if (current == goal) {
      break;
    }
    const int cx = static_cast<int>(current % size_x);
    const int cy = static_cast<int>(current / size_x);
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dx[k];
      const int ny = cy + dy[k];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const unsigned int next = static_cast<unsigned int>(ny) * size_x + nx;
      if (closed[next] || !passable(next)) {
        continue;
      }
      // No diagonal step between two blocked side cells.
      if (k >= 4 && (!passable(cy * size_x + nx) || !passable(ny * size_x + cx))) {
        continue;
      }
      const double candidate = d + step_cost(next, length[k]);
      if (candidate < distance[next]) {
        distance[next] = candidate;
        parent[next] = current;
        open.emplace(candidate, next);
      }
    }
  }
  if (!closed[goal]) {
    return {};
  }

  std::vector<unsigned int> cells_on_path;
  for (unsigned int index = goal; index != cells; index = parent[index]) {
    cells_on_path.push_back(index);
    if (index == start) {
      break;
    }
  }
  std::reverse(cells_on_path.begin(), cells_on_path.end());
  return cells_on_path;
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "DijkstraPlanner plans in " + global_frame_ + " only; got start in " +
            start.header.frame_id + " and goal in " + goal.header.frame_id);
  }

  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  unsigned int sx, sy, gx, gy;
  std::vector<unsigned int> cells_on_path;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy)) {
      throw nav2_core::PlannerException("DijkstraPlanner: the start is outside the costmap");
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
      throw nav2_core::PlannerException("DijkstraPlanner: the goal is outside the costmap");
    }
    cells_on_path = search(
      costmap_->getCharMap(), costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
      costmap_->getIndex(sx, sy), costmap_->getIndex(gx, gy), allow_unknown_,
      static_cast<unsigned char>(lethal_cost_), cost_factor_);
    if (cells_on_path.empty()) {
      throw nav2_core::PlannerException(
              "DijkstraPlanner: no path to the goal (it is blocked or unreachable)");
    }
    for (const unsigned int index : cells_on_path) {
      unsigned int mx, my;
      costmap_->indexToCells(index, mx, my);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      costmap_->mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
      path.poses.push_back(pose);
    }
  }

  // The ends are the requested poses, not their cell centres.
  path.poses.front().pose.position = start.pose.position;
  if (path.poses.size() == 1) {
    path.poses.push_back(path.poses.front());
  }
  path.poses.back().pose.position = goal.pose.position;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(b.y - a.y, b.x - a.x));
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
  path.poses.back().pose.orientation = goal.pose.orientation;

  if (use_smoother_) {
    smooth(path);
  }
  return path;
}

void DijkstraPlanner::smooth(nav_msgs::msg::Path & path)
{
  if (!smooth_client_ || !smooth_client_->action_server_is_ready()) {
    return;
  }
  nav2_msgs::action::SmoothPath::Goal request;
  request.path = path;
  request.check_for_collisions = false;
  request.smoother_id = smoother_id_;
  request.max_smoothing_duration.sec = 1;
  auto future = smooth_client_->async_send_goal(request);
  if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    return;
  }
  auto handle = future.get();
  if (!handle) {
    return;
  }
  auto result = smooth_client_->async_get_result(handle);
  if (result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    auto wrapped = result.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && !wrapped.result->path.poses.empty()) {
      path = wrapped.result->path;
    }
  }
}

}  // namespace robot_lab_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_lab_planning::DijkstraPlanner, nav2_core::GlobalPlanner)
