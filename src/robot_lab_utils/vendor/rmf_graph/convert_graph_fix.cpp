/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Extracted official Humble graph serializer. Fix exit-event null dereference
// and exit-orientation lookup; scoped interposition for the Python adapter.
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_building_map_msgs/msg/graph.hpp>
#include <rmf_building_map_msgs/msg/graph_node.hpp>
#include <rmf_building_map_msgs/msg/graph_edge.hpp>
#include <rmf_building_map_msgs/msg/param.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
namespace rmf_traffic_ros2 {
class RobotLabEventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;
  using GraphParamMsg = rmf_building_map_msgs::msg::Param;

  RobotLabEventPhaseFactory(
    const std::string& prefix,
    std::vector<GraphParamMsg>& edge_params)
  : _prefix(std::move(prefix)),
    _edge_params(edge_params)
  {
    // Do nothing
  }

  void execute(const Dock& dock) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_dock_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(dock.dock_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_dock_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(dock.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const DoorOpen& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_open_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_open_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const DoorClose& close) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_close_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_door_close_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(close.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftSessionBegin& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftMove& move) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(move.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(move.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_move_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(move.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftDoorOpen& open) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(open.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_door_open_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(open.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const LiftSessionEnd& close) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_lift_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.lift_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_floor_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0)
      .value_string(close.floor_name())
      .value_bool(false));

    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_lift_end_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(close.duration().count())
      .value_string("")
      .value_bool(false));
  }

  void execute(const Wait& wait) final
  {
    _edge_params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name(_prefix + "_wait_duration")
      .type(GraphParamMsg::TYPE_INT)
      .value_int(0)
      .value_float(wait.duration().count())
      .value_string("")
      .value_bool(false));
  }

private:
  std::string _prefix;
  std::vector<GraphParamMsg>& _edge_params;
};

//==============================================================================
std::unique_ptr<rmf_building_map_msgs::msg::Graph> convert(
  const rmf_traffic::agv::Graph& graph, const std::string& fleet_name)
{
  using GraphMsg = rmf_building_map_msgs::msg::Graph;
  using GraphNodeMsg = rmf_building_map_msgs::msg::GraphNode;
  using GraphEdgeMsg = rmf_building_map_msgs::msg::GraphEdge;
  using GraphParamMsg = rmf_building_map_msgs::msg::Param;

  if (fleet_name.empty())
    return nullptr;

  const std::size_t n_waypoints = graph.num_waypoints();
  const std::size_t n_lanes = graph.num_lanes();

  if (n_waypoints == 0 || n_lanes == 0)
    return nullptr;

  std::vector<GraphNodeMsg> vertices;
  std::vector<GraphEdgeMsg> edges;
  // Populate vertices
  for (std::size_t i = 0; i < n_waypoints; ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    const auto& loc = wp.get_location();
    std::vector<GraphParamMsg> params;
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("map_name")
      .type(GraphParamMsg::TYPE_STRING)
      .value_int(0)
      .value_float(0.0)
      .value_string(wp.get_map_name())
      .value_bool(false));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_holding_point")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_holding_point()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_passthrough_point")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_passthrough_point()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_parking_spot")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_parking_spot()));
    params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
      .name("is_charger")
      .type(GraphParamMsg::TYPE_BOOL)
      .value_int(0)
      .value_float(0.0)
      .value_string("")
      .value_bool(wp.is_charger()));
    const std::string wp_name = wp.name() ? *wp.name() : "";
    vertices.emplace_back(rmf_building_map_msgs::build<GraphNodeMsg>()
      .x(loc[0])
      .y(loc[1])
      .name(wp_name)
      .params(std::move(params)));
  }
  // Populate edges
  for (std::size_t i = 0; i < n_lanes; ++i)
  {
    const auto& lane = graph.get_lane(i);
    // All lanes in rmf_traffic::agv::Graph are unidirectional
    std::vector<GraphParamMsg> params;
    const auto& properties = graph.get_lane(i).properties();
    if (properties.speed_limit().has_value())
    {
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("speed_limit")
        .type(GraphParamMsg::TYPE_DOUBLE)
        .value_int(0)
        .value_float(properties.speed_limit().value())
        .value_string("")
        .value_bool(false));
    }
    // Serialize events for this lane
    const auto entry_event = lane.entry().event();
    if (entry_event != nullptr)
    {
      RobotLabEventPhaseFactory factory("entry", params);
      entry_event->execute(factory);
    }
    const auto* entry_orientation = lane.entry().orientation_constraint();
    if (entry_orientation != nullptr)
    {
      const auto& forward = Eigen::Vector2d::UnitX();
      Eigen::Vector3d pos = {1.0, 0.0, 0.0};
      entry_orientation->apply(pos, forward);
      std::string dir = std::abs((pos[2] - 0.0)) <
        1e-3 ? "forward" : "backward";
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("entry_orientation_constraint")
        .type(GraphParamMsg::TYPE_STRING)
        .value_int(0)
        .value_float(0.0)
        .value_string(std::move(dir))
        .value_bool(false));
    }
    const auto exit_event = lane.exit().event();
    if (exit_event != nullptr)
    {
      RobotLabEventPhaseFactory factory("exit", params);
      exit_event->execute(factory);
    }
    const auto* exit_orientation = lane.exit().orientation_constraint();
    if (exit_orientation != nullptr)
    {
      const auto& forward = Eigen::Vector2d::UnitX();
      Eigen::Vector3d pos = {1.0, 0.0, 0.0};
      exit_orientation->apply(pos, forward);
      std::string dir = std::abs((pos[2] - 0.0)) <
        1e-3 ? "forward" : "backward";
      params.emplace_back(rmf_building_map_msgs::build<GraphParamMsg>()
        .name("exit_orientation_constraint")
        .type(GraphParamMsg::TYPE_STRING)
        .value_int(0)
        .value_float(0.0)
        .value_string(std::move(dir))
        .value_bool(false));
    }
    // Add vertices for this lane
    edges.emplace_back(
      rmf_building_map_msgs::build<GraphEdgeMsg>()
      .v1_idx(lane.entry().waypoint_index())
      .v2_idx(lane.exit().waypoint_index())
      .params(std::move(params))
      .edge_type(GraphEdgeMsg::EDGE_TYPE_UNIDIRECTIONAL)
    );
  }

  std::unique_ptr<GraphMsg> msg = std::make_unique<GraphMsg>(
    rmf_building_map_msgs::build<GraphMsg>()
    .name(fleet_name)
    .vertices(std::move(vertices))
    .edges(std::move(edges))
    .params({})
  );

  return msg;
}

} // namespace rmf_traffic_ros2
