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


#include <velocity_planner/velocity_graph.hpp>
#include <iostream>
#include <algorithm>
#include <map>
#include <vector>

namespace velocity_planner
{
VelocityGraph::VelocityGraph(
  hermite_path_msgs::msg::HermitePathStamped data, double velocity_resoluation,
  double maximum_accerelation, double minimum_accerelation, double maximum_velocity)
: generator_(0.0)
{
  path_ = data;
  planning_succeed_ = false;
  minimum_acceleration_ = minimum_accerelation;
  maximum_acceleration_ = maximum_accerelation;
  velocity_resoluation_ = velocity_resoluation;
  maximum_velocity_ = maximum_velocity;
  path_length_ = generator_.getLength(data.path, 200);
  std::map<double, std::vector<Node>> nodes;
  bool stop_flag_founded = false;
  nodes[0.0] = makeStartNodes();
  for (auto itr = data.reference_velocity.begin(); itr != data.reference_velocity.end(); itr++) {
    if (itr->stop_flag) {
      nodes[itr->t] = makeEndNode(itr->t);
      stop_flag_founded = true;
      break;
    } else {
      nodes[itr->t] = makeNodes(*itr);
    }
  }
  if (!stop_flag_founded) {
    nodes[1.0] = makeEndNode(1.0);
  }
  boost::optional<std::vector<Edge>> edges = makeEdges(nodes);
  if (edges) {
    buildVelocityGraph(nodes, edges.get());
    plan();
  }
}

boost::optional<std::vector<hermite_path_msgs::msg::ReferenceVelocity>> VelocityGraph::getPlan()
{
  if (!planning_succeed_) {
    return boost::none;
  }
  return result_;
}

void VelocityGraph::plan()
{
  std::vector<VelocityGraphData::vertex_descriptor> parents(boost::num_vertices(data_));
  std::vector<double> weights(boost::num_vertices(data_));
  std::vector<Plan> plans;
  for (auto id_itr = start_node_ids_.begin(); id_itr != start_node_ids_.end(); id_itr++) {
    VelocityGraphData::vertex_descriptor from = vertex_dict_[*id_itr];
    VelocityGraphData::vertex_descriptor to = vertex_dict_[end_node_id_];
    auto visitor = boost::weight_map(boost::get(&Edge::weight, data_))
      .predecessor_map(&parents[0])
      .distance_map(&weights[0]).visitor(AstarGoalVisitor(to));
    try {
      boost::astar_search_tree(data_, from, HeuristicFunc(to, data_), visitor);
    } catch (found_goal fg) {
      if (parents[to] != to) {
        Plan p;
        for (auto v = to;; v = parents[v]) {
          hermite_path_msgs::msg::ReferenceVelocity vel;
          vel = data_[v].vel;
          p.plan.push_front(vel);
          if (v == from || plan_length_ < static_cast<int>(p.plan.size())) {
            break;
          }
        }
        if (plan_length_ == static_cast<int>(p.plan.size())) {
          p.total_weights = weights[to];
          plans.push_back(p);
        }
      }
    }
  }
  if (plans.size() == 0) {
    planning_succeed_ = false;
    reason_ = "failed in all velocity plans";
  } else {
    planning_succeed_ = true;
    result_.clear();
    std::sort(
      plans.begin(), plans.end(), [](const auto & a, const auto & b) {
        return a.total_weights < b.total_weights;
      });
    Plan p = plans[0];
    for (auto itr = p.plan.begin(); itr != p.plan.end(); itr++) {
      result_.push_back(*itr);
    }
    calculateAcceleration();
  }
}

void VelocityGraph::calculateAcceleration()
{
  std::vector<double> accels;
  for (int i = 0; i < (static_cast<int>(result_.size()) - 1); i++) {
    double v0 = result_[i].linear_velocity;
    double v1 = result_[i + 1].linear_velocity;
    double a = (v1 * v1 - v0 * v0) / (2 * path_length_);
    accels.push_back(a);
  }
  std::sort(accels.begin(), accels.end(), [](const auto & a, const auto & b) {return a > b;});
  planned_maximum_acceleration_ = accels[0];
  std::sort(accels.begin(), accels.end(), [](const auto & a, const auto & b) {return a < b;});
  planned_minimum_acceleration_ = accels[0];
}

std::vector<Node> VelocityGraph::makeStartNodes()
{
  assert(path_.reference_velocity.size() != 0);
  hermite_path_msgs::msg::ReferenceVelocity v0 = path_.reference_velocity[0];
  v0.t = 0.0;
  std::vector<Node> ret = makeNodes(v0);
  for (auto itr = ret.begin(); itr != ret.end(); itr++) {
    start_node_ids_.push_back(itr->id);
  }
  return ret;
}

std::vector<Node> VelocityGraph::makeEndNode(double t)
{
  std::vector<Node> ret;
  Node end_node;
  end_node.vel.t = t;
  end_node.vel.linear_velocity = 0.0;
  end_node.id = boost::uuids::random_generator()();
  end_node_id_ = end_node.id;
  ret.push_back(end_node);
  return ret;
}

void VelocityGraph::buildVelocityGraph(
  std::map<double, std::vector<Node>> nodes, std::vector<Edge> edges)
{
  vertex_dict_.clear();
  data_.clear();
  for (auto it = nodes.begin(); it != nodes.end(); it++) {
    for (auto n = it->second.begin(); n != it->second.end(); n++) {
      vertex_dict_[n->id] = add_vertex(data_);
      data_[vertex_dict_[n->id]] = *n;
    }
  }
  for (auto it = edges.begin(); it != edges.end(); it++) {
    VelocityGraphData::edge_descriptor e;
    VelocityGraphData::vertex_descriptor v0 = vertex_dict_[it->before_node_id];
    VelocityGraphData::vertex_descriptor v1 = vertex_dict_[it->after_node_id];
    bool inserted = false;
    boost::tie(e, inserted) = add_edge(v0, v1, data_);
    data_[e] = *it;
  }
}

boost::optional<std::vector<Edge>> VelocityGraph::makeEdges(
  std::map<double, std::vector<Node>> nodes)
{
  std::vector<Edge> edges;
  std::vector<double> t_values;
  for (auto it = nodes.begin(); it != nodes.end(); it++) {
    t_values.push_back(it->first);
  }
  std::sort(t_values.begin(), t_values.end(), [](const auto & a, const auto & b) {return a < b;});
  plan_length_ = t_values.size();
  for (int i = 0; i < (static_cast<int>(t_values.size()) - 1); i++) {
    bool connection_finded = false;
    std::vector<Node> before_nodes = nodes[t_values[i]];
    std::vector<Node> after_nodes = nodes[t_values[i + 1]];
    double l = (t_values[i + 1] - t_values[i]) * path_length_;
    for (auto before_itr = before_nodes.begin(); before_itr != before_nodes.end(); before_itr++) {
      for (auto after_itr = after_nodes.begin(); after_itr != after_nodes.end(); after_itr++) {
        double v0 = before_itr->vel.linear_velocity;
        double v1 = after_itr->vel.linear_velocity;
        double a = (v1 * v1 - v0 * v0) / (2 * l);
        if (
          a > minimum_acceleration_ && a < maximum_acceleration_ &&
          std::fabs(v0) <= maximum_velocity_ && std::fabs(v1) <= maximum_velocity_)
        {
          connection_finded = true;
          Edge edge;
          edge.before_node_id = before_itr->id;
          edge.after_node_id = after_itr->id;
          edge.linear_accerelation = a;
          edge.weight =
            std::fabs(std::fabs(v0) - maximum_velocity_) +
            std::fabs(std::fabs(v1) - maximum_velocity_);
          edges.push_back(edge);
        }
      }
    }
    if (!connection_finded) {
      reason_ = "failed to build edge from t=" + std::to_string(t_values[i]) +
        " to t=" + std::to_string(t_values[i + 1]);
      return boost::none;
    }
  }
  return edges;
}

std::vector<Node> VelocityGraph::makeNodes(hermite_path_msgs::msg::ReferenceVelocity vel)
{
  std::vector<Node> ret;
  double v = 0.0;
  if (velocity_resoluation_ < vel.linear_velocity) {
    int count = 0;
    while (v < vel.linear_velocity) {
      v = velocity_resoluation_ * count;
      Node n;
      n.vel.t = vel.t;
      n.vel.linear_velocity = v;
      n.id = boost::uuids::random_generator()();
      ret.push_back(n);
      count++;
    }
  } else {
    Node n;
    n.vel.t = vel.t;
    n.vel.linear_velocity = vel.linear_velocity;
    n.id = boost::uuids::random_generator()();
    ret.push_back(n);
  }
  return ret;
}
}  // namespace velocity_planner
