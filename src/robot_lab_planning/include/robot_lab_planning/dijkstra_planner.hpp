#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

namespace robot_lab_planning
{

// Uniform-cost (Dijkstra) search over the global costmap, eight-connected.
// It expands by accumulated cost only, with no heuristic, so it is the
// exact cost-optimal grid path for the edge costs below; NavFn computes a
// potential field instead and follows its gradient.
class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  // The search itself, on a row-major cost grid (exposed for tests).
  // Returns the cell indices from start to goal, or empty when none.
  static std::vector<unsigned int> search(
    const unsigned char * costs, unsigned int size_x, unsigned int size_y,
    unsigned int start, unsigned int goal, bool allow_unknown,
    unsigned char lethal_cost, double cost_factor);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_, name_;

  bool allow_unknown_;
  int lethal_cost_;
  double cost_factor_;
  bool use_smoother_;
  std::string smoother_id_;

  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;

  void smooth(nav_msgs::msg::Path & path);
};

}  // namespace robot_lab_planning

#endif  // DIJKSTRA_PLANNER_HPP
