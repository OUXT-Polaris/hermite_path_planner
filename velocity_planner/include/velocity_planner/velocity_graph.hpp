// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VELOCITY_PLANNER__VELOCITY_GRAPH_HPP_
#define VELOCITY_PLANNER__VELOCITY_GRAPH_HPP_

#include <hermite_path_planner/hermite_path_generator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <hermite_path_msgs/msg/reference_velocity.hpp>
#include <map>
#include <string>
#include <vector>
#include <deque>

namespace velocity_planner
{
struct Node
{
  hermite_path_msgs::msg::ReferenceVelocity vel;
  boost::uuids::uuid id;
};

struct Edge
{
  boost::uuids::uuid before_node_id;
  boost::uuids::uuid after_node_id;
  double linear_accerelation;
  double weight;
};

struct Graph
{
};

struct Plan
{
  std::deque<hermite_path_msgs::msg::ReferenceVelocity> plan;
  double total_weights;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, Node, Edge, Graph>
  VelocityGraphData;

class HeuristicFunc : public boost::astar_heuristic<VelocityGraphData, double>
{
public:
  using Vertex = VelocityGraphData::vertex_descriptor;

  explicit HeuristicFunc(Vertex goal, VelocityGraphData & g)
  : goal_(goal), graph_(g)
  {
  }

  double operator()(Vertex u) const
  {
    return graph_[u].vel.linear_velocity;
  }

private:
  Vertex goal_;
  VelocityGraphData & graph_;
};

struct found_goal {};
class AstarGoalVisitor : public boost::default_astar_visitor
{
public:
  explicit AstarGoalVisitor(VelocityGraphData::vertex_descriptor goal)
  : m_goal(goal) {}

  template<class Graph>
  void examine_vertex(VelocityGraphData::vertex_descriptor u, Graph &)
  {
    if (u == m_goal) {
      throw found_goal();
    }
  }

private:
  VelocityGraphData::vertex_descriptor m_goal;
};

class VelocityGraph
{
public:
  VelocityGraph(
    hermite_path_msgs::msg::HermitePathStamped data, double velocity_resoluation,
    double maximum_acceleration, double minimum_acceleration, double maximum_velocity);
  boost::optional<std::vector<hermite_path_msgs::msg::ReferenceVelocity>> getPlan();
  std::string getReason() {return reason_;}
  double getPlannedMaximumAcceleration() {return planned_maximum_acceleration_;}
  double getPlannedMinimumAcceleration() {return planned_minimum_acceleration_;}

private:
  void plan();
  std::vector<Node> makeNodes(hermite_path_msgs::msg::ReferenceVelocity vel);
  std::vector<Node> makeStartNodes();
  std::vector<Node> makeEndNode(double t);
  boost::optional<std::vector<Edge>> makeEdges(std::map<double, std::vector<Node>> nodes);
  void buildVelocityGraph(std::map<double, std::vector<Node>> nodes, std::vector<Edge> edges);
  VelocityGraphData data_;
  double maximum_velocity_;
  double velocity_resoluation_;
  double maximum_acceleration_;
  double minimum_acceleration_;
  double path_length_;
  hermite_path_msgs::msg::HermitePathStamped path_;
  hermite_path_planner::HermitePathGenerator generator_;
  bool planning_succeed_;
  std::string reason_;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> result_;
  std::vector<boost::uuids::uuid> start_node_ids_;
  boost::uuids::uuid end_node_id_;
  std::map<boost::uuids::uuid, VelocityGraphData::vertex_descriptor> vertex_dict_;
  void calculateAcceleration();
  int plan_length_;
  double planned_maximum_acceleration_;
  double planned_minimum_acceleration_;
};
}  // namespace velocity_planner

#endif  // VELOCITY_PLANNER__VELOCITY_GRAPH_HPP_
