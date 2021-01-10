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

#include <hermite_path_planner/hermite_path_generator.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <color_names/color_names.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>

namespace hermite_path_planner
{
HermitePathGenerator::HermitePathGenerator(double robot_width) {robot_width_ = robot_width;}

double HermitePathGenerator::getMaximumCurvature(
  hermite_path_msgs::msg::HermitePath path,
  int resolution)
{
  std::vector<double> curvatures;
  double step_size = 1.0 / static_cast<double>(resolution);
  for (int i = 0; i < (resolution + 1); i++) {
    double t = step_size * static_cast<double>(i);
    curvatures.push_back(getCurvature(path, t));
  }
  return *std::max_element(curvatures.begin(), curvatures.end());
}

double HermitePathGenerator::getCurvature(hermite_path_msgs::msg::HermitePath path, double t)
{
  double t2 = t * t;
  double x_dot = 3 * path.ax * t2 + 2 * path.bx * t + path.cx;
  double x_dot_dot = 6 * path.ax * t + 2 * path.bx;
  double y_dot = 3 * path.ay * t2 + 2 * path.by * t + path.cy;
  double y_dot_dot = 6 * path.ay * t + 2 * path.by;
  return (x_dot * y_dot_dot - x_dot_dot * y_dot) / std::pow(x_dot * x_dot + y_dot * y_dot, 1.5);
}

double HermitePathGenerator::getReferenceVelocity(
  hermite_path_msgs::msg::HermitePathStamped path, double t)
{
  if (path.reference_velocity.size() == 0) {
    return 0.0;
  }
  if (path.reference_velocity.size() == 1) {
    return path.reference_velocity[0].linear_velocity;
  }
  double l = getLength(path.path, 200);
  std::sort(
    path.reference_velocity.begin(), path.reference_velocity.end(),
    [](const auto & a, const auto & b) {return a.t < b.t;});
  if (path.reference_velocity.begin()->t > t) {
    double diff_l = (path.reference_velocity[1].t - path.reference_velocity[0].t) * l;
    double a = (std::pow(path.reference_velocity[1].linear_velocity, 2) -
      std::pow(path.reference_velocity[0].linear_velocity, 2)) /
      (2 * diff_l);
    double diff_l_target = (path.reference_velocity[0].t - t) * l;
    double v2 = std::pow(path.reference_velocity[1].linear_velocity, 2) - 2 * a * diff_l_target;
    if (v2 <= 0.0) {
      return 0.0;
    } else {
      return std::sqrt(v2);
    }
  }
  /*
  if (path.reference_velocity.end()->t > t) {
    size_t end_ref_vel_index = path.reference_velocity.size() - 1;
    double diff_l = (path.reference_velocity[end_ref_vel_index].t -
      path.reference_velocity[end_ref_vel_index - 1].t) *
      l;
    double a = (std::pow(path.reference_velocity[end_ref_vel_index].linear_velocity, 2) -
      std::pow(path.reference_velocity[end_ref_vel_index - 1].linear_velocity, 2)) /
      (2 * diff_l);
    double diff_l_target = (t - path.reference_velocity[end_ref_vel_index].t) * l;
    double v2 = std::pow(path.reference_velocity[end_ref_vel_index].linear_velocity, 2) +
      2 * a * diff_l_target;
    if (v2 <= 0.0) {
      return 0.0;
    } else {
      return std::sqrt(v2);
    }
  }
  */
  for (int i = 0; i < static_cast<int>(path.reference_velocity.size()) - 1; i++) {
    if (path.reference_velocity[i + 1].t >= t && t >= path.reference_velocity[i].t) {
      double diff_l = (path.reference_velocity[i + 1].t - path.reference_velocity[i].t) * l;
      double a = (std::pow(path.reference_velocity[i + 1].linear_velocity, 2) -
        std::pow(path.reference_velocity[i].linear_velocity, 2)) /
        (2 * diff_l);
      double diff_l_target = (t - path.reference_velocity[i].t) * l;
      double v2 = std::pow(path.reference_velocity[i].linear_velocity, 2) + 2 * a * diff_l_target;
      if (v2 <= 0.0) {
        return 0.0;
      } else {
        return std::sqrt(v2);
      }
    }
  }
  return 0.0;
}

boost::optional<double> HermitePathGenerator::getLateralDistanceInFrenetCoordinate(
  hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p)
{
  boost::optional<double> t = getNormalizedLongitudinalDistanceInFrenetCoordinate(path, p);
  if (!t) {
    return boost::none;
  } else if (t.get() < 0.0 || 1.0 < t.get()) {
    return boost::none;
  }
  geometry_msgs::msg::Point point = getPointOnHermitePath(path, t.get());
  return std::sqrt(std::pow(point.x - p.x, 2) + std::pow(point.y - p.y, 2));
}

boost::optional<double> HermitePathGenerator::getLongitudinalDistanceInFrenetCoordinate(
  hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p, int resolution)
{
  boost::optional<double> t = getNormalizedLongitudinalDistanceInFrenetCoordinate(path, p);
  if (!t) {
    return boost::none;
  } else if (t.get() < 0.0 || 1.0 < t.get()) {
    return boost::none;
  }
  double l = getLength(path, resolution);
  return l * t.get();
}

boost::optional<double> HermitePathGenerator::getNormalizedLongitudinalDistanceInFrenetCoordinate(
  hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p)
{
  auto func =
    [](hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point point, double t) {
      double t3 = std::pow(t, 3);
      double t2 = std::pow(t, 2);
      double x_term = std::pow(point.x - path.ax * t3 - path.bx * t2 - path.cx * t - path.dx, 2);
      double y_term = std::pow(point.y - path.ay * t3 - path.by * t2 - path.cy * t - path.dy, 2);
      double ret = x_term + y_term;
      return ret;
    };
  auto get_newton_step_size_func =
    [](hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point point, double t) {
      double t3 = std::pow(t, 3);
      double t2 = std::pow(t, 2);
      double x_term = std::pow(point.x - path.ax * t3 - path.bx * t2 - path.cx * t - path.dx, 2);
      double y_term = std::pow(point.y - path.ay * t3 - path.by * t2 - path.cy * t - path.dy, 2);
      double x_term_diff = 2 * (point.x - path.ax * t3 - path.bx * t2 - path.cx * t - path.dx) *
        (-3 * path.ax * t2 - 2 * path.bx * t - path.cx);
      double y_term_diff = 2 * (point.y - path.ay * t3 - path.by * t2 - path.cy * t - path.dy) *
        (-3 * path.ay * t2 - 2 * path.by * t - path.cy);
      double ret = (x_term + y_term) / (x_term_diff + y_term_diff);
      return ret;
    };

  constexpr int initial_resolution = 30;
  constexpr int max_iteration = 30;
  constexpr double torelance = 0.001;

  double step_size = static_cast<double>(1.0) / static_cast<double>(initial_resolution);
  double ret = 0.0;
  std::vector<double> initial_value_candidates(initial_resolution);
  std::vector<double> initial_errors(initial_resolution);
  for (int i = 0; i < initial_resolution; i++) {
    initial_value_candidates[i] = (0.5 + static_cast<double>(i)) * step_size;
    initial_errors[i] = std::fabs(func(path, p, initial_value_candidates[i]));
  }
  std::vector<double>::iterator iter =
    std::min_element(initial_errors.begin(), initial_errors.end());
  size_t index = std::distance(initial_errors.begin(), iter);
  ret = initial_value_candidates[index];
  std::vector<double> errors;
  std::vector<double> t_values;
  for (int i = 0; i < max_iteration; i++) {
    double error = func(path, p, ret);
    if (std::fabs(error) < torelance) {
      return ret;
    }
    t_values.push_back(ret);
    errors.push_back(error);
    ret = ret - get_newton_step_size_func(path, p, ret);
  }
  std::vector<double>::iterator min_iter = std::min_element(errors.begin(), errors.end());
  size_t value_index = std::distance(errors.begin(), min_iter);
  ret = t_values[value_index];
  return ret;
}

double HermitePathGenerator::getLength(hermite_path_msgs::msg::HermitePath path, int resolution)
{
  double ret = 0.0;
  std::vector<geometry_msgs::msg::Point> points = getPointsOnHermitePath(path, resolution);
  for (int i = 0; i < (resolution - 1); i++) {
    double length_segment = std::sqrt(
      std::pow(points[i].x - points[i + 1].x, 2) + std::pow(points[i].y - points[i + 1].y, 2));
    ret = length_segment + ret;
  }
  return ret;
}

double HermitePathGenerator::calculateNewtonMethodStepSize(
  hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point center, double radius,
  double t)
{
  double t3 = t * t * t;
  double t2 = t * t;
  double f = std::pow((path.ax * t3 + path.bx * t2 + path.cx * t + path.dx - center.x), 2) +
    std::pow((path.ay * t3 + path.by * t2 + path.cy * t + path.dy - center.y), 2) -
    radius * radius;
  double term_x = 2 * (path.ax * t3 + path.bx * t2 + path.cx * t + path.dx - center.x) *
    (3 * path.ax * t2 + 2 * path.bx * t + path.cx);
  double term_y = 2 * (path.ay * t3 + path.by * t2 + path.cy * t + path.dy - center.y) *
    (3 * path.ay * t2 + 2 * path.by * t + path.cy);
  return f / (term_x + term_y);
}

boost::optional<double> HermitePathGenerator::checkFirstCollisionWithCircle(
  hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point center, double radius)
{
  constexpr int max_iteration = 30;
  constexpr double torelance = 0.01;
  auto dist = getNormalizedLongitudinalDistanceInFrenetCoordinate(path, center);
  if (!dist) {
    return boost::none;
  }
  double length = getLength(path, 30);
  double ret = dist.get() + radius / length;
  for (int i = 0; i < max_iteration; i++) {
    geometry_msgs::msg::Point p = getPointOnHermitePath(path, ret);
    double error = std::sqrt(std::pow(p.x - center.x, 2) + std::pow(p.y - center.y, 2)) - radius;
    if (std::fabs(error) < torelance) {
      return ret;
    }
    double diff = calculateNewtonMethodStepSize(path, center, radius, ret);
    ret = ret - diff;
  }
  return boost::none;
}

hermite_path_msgs::msg::HermitePath HermitePathGenerator::generateHermitePath(
  geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose goal)
{
  double goal_distance =
    std::sqrt(
    std::pow(goal.position.x - start.position.x, 2) +
    std::pow(goal.position.y - start.position.y, 2));
  /*
  double diff_x = goal.position.x - start.position.x;
  double diff_y = goal.position.y - start.position.y;
  double diff_theta = std::fabs(std::atan2(diff_y, diff_x));
  double ratio = sigmoid(1.0, diff_theta) * 3.0;
  */
  const int resolution = 20;
  double step_size = 5.0 / static_cast<double>(resolution);
  std::vector<double> path_lengths;
  std::vector<hermite_path_msgs::msg::HermitePath> path_list;
  for (int i = 0; i < (resolution + 1); i++) {
    for (int m = 0; m < (resolution + 1); m++) {
      double ratio_start = step_size * static_cast<double>(i) + 0.5;
      double ratio_goal = step_size * static_cast<double>(i) + 0.5;
      auto path = generateHermitePath(
        start, goal,
        ratio_start * goal_distance, ratio_goal * goal_distance);
      path_lengths.push_back(getLength(path, 100));
      // path_lengths.push_back(getMaximumCurvature(path,50));
      path_list.push_back(path);
    }
  }
  auto itr = std::min_element(path_lengths.begin(), path_lengths.end());
  size_t index = std::distance(path_lengths.begin(), itr);
  return path_list[index];
}

hermite_path_msgs::msg::HermitePath HermitePathGenerator::generateHermitePath(
  geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose goal, double start_vector_magnitude,
  double end_vector_magnitude)
{
  hermite_path_msgs::msg::HermitePath path;
  geometry_msgs::msg::Vector3 start_vector = getVectorFromPose(start, start_vector_magnitude);
  geometry_msgs::msg::Vector3 goal_vector = getVectorFromPose(goal, end_vector_magnitude);
  path.ax = 2 * start.position.x - 2 * goal.position.x + start_vector.x + goal_vector.x;
  path.ay = 2 * start.position.y - 2 * goal.position.y + start_vector.y + goal_vector.y;
  path.bx = -3 * start.position.x + 3 * goal.position.x - 2 * start_vector.x - goal_vector.x;
  path.by = -3 * start.position.y + 3 * goal.position.y - 2 * start_vector.y - goal_vector.y;
  path.cx = start_vector.x;
  path.cy = start_vector.y;
  path.dx = start.position.x;
  path.dy = start.position.y;
  return path;
}

std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getPointsOnHermitePath(
  hermite_path_msgs::msg::HermitePath path, int resolution, double max_t)
{
  std::vector<geometry_msgs::msg::Point> p;
  double step_size = max_t / static_cast<double>(resolution);
  for (int i = 0; i < (resolution + 1); i++) {
    double t = step_size * static_cast<double>(i);
    p.push_back(getPointOnHermitePath(path, t));
  }
  return p;
}

std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getLeftBounds(
  hermite_path_msgs::msg::HermitePath path, int resolution)
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = 1.0 / static_cast<double>(resolution);
  for (int i = 0; i < (resolution + 1); i++) {
    double t = step_size * static_cast<double>(i);
    geometry_msgs::msg::Vector3 vec = getNormalVector(path, t);
    double theta = std::atan2(vec.y, vec.x);
    geometry_msgs::msg::Point p = getPointOnHermitePath(path, t);
    geometry_msgs::msg::Point point;
    point.x = p.x - 0.5 * robot_width_ * std::cos(theta);
    point.y = p.y - 0.5 * robot_width_ * std::sin(theta);
    points.push_back(point);
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getRightBounds(
  hermite_path_msgs::msg::HermitePath path, int resolution)
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = 1.0 / static_cast<double>(resolution);
  for (int i = 0; i < (resolution + 1); i++) {
    double t = step_size * static_cast<double>(i);
    geometry_msgs::msg::Vector3 vec = getNormalVector(path, t);
    double theta = std::atan2(vec.y, vec.x);
    geometry_msgs::msg::Point p = getPointOnHermitePath(path, t);
    geometry_msgs::msg::Point point;
    point.x = p.x + 0.5 * robot_width_ * std::cos(theta);
    point.y = p.y + 0.5 * robot_width_ * std::sin(theta);
    points.push_back(point);
  }
  return points;
}

visualization_msgs::msg::Marker HermitePathGenerator::getBoundsPolygon(
  hermite_path_msgs::msg::HermitePathStamped path, int resolution, double z_offset)
{
  std_msgs::msg::ColorRGBA color = color_names::makeColorMsg("skyblue", 0.8);
  visualization_msgs::msg::Marker marker;
  marker.header = path.header;
  marker.ns = "polygon";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  std::vector<geometry_msgs::msg::Point> left_bounds = getLeftBounds(path.path, resolution);
  std::vector<geometry_msgs::msg::Point> right_bounds = getRightBounds(path.path, resolution);
  int num_sections = left_bounds.size() - 1;
  assert(left_bounds.size() == right_bounds.size());
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = color;
  for (int i = 0; i < num_sections; i++) {
    geometry_msgs::msg::Point pr_0 = right_bounds[i];
    pr_0.z = z_offset;
    geometry_msgs::msg::Point pl_0 = left_bounds[i];
    pl_0.z = z_offset;
    geometry_msgs::msg::Point pr_1 = right_bounds[i + 1];
    pr_1.z = z_offset;
    geometry_msgs::msg::Point pl_1 = left_bounds[i + 1];
    pl_1.z = z_offset;
    marker.points.push_back(pr_0);
    marker.points.push_back(pl_0);
    marker.points.push_back(pr_1);
    marker.colors.push_back(color);
    marker.points.push_back(pl_0);
    marker.points.push_back(pl_1);
    marker.points.push_back(pr_1);
    marker.colors.push_back(color);
  }
  return marker;
}

geometry_msgs::msg::Point HermitePathGenerator::getPointOnHermitePath(
  hermite_path_msgs::msg::HermitePath path, double t)
{
  geometry_msgs::msg::Point p;
  p.z = 0;
  p.x = path.ax * std::pow(t, 3) + path.bx * std::pow(t, 2) + path.cx * t + path.dx;
  p.y = path.ay * std::pow(t, 3) + path.by * std::pow(t, 2) + path.cy * t + path.dy;
  return p;
}

geometry_msgs::msg::Vector3 HermitePathGenerator::getVectorFromPose(
  geometry_msgs::msg::Pose pose, double magnitude)
{
  geometry_msgs::msg::Vector3 dir =
    quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
  geometry_msgs::msg::Vector3 vector;
  vector.x = magnitude * std::cos(dir.z);
  vector.y = magnitude * std::sin(dir.z);
  vector.z = 0;
  return vector;
}

geometry_msgs::msg::Vector3 HermitePathGenerator::getTangentVector(
  hermite_path_msgs::msg::HermitePath path, double t)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = 3 * path.ax * t * t + 2 * path.bx * t + path.cx;
  vec.y = 3 * path.ay * t * t + 2 * path.by * t + path.cy;
  return vec;
}

geometry_msgs::msg::Vector3 HermitePathGenerator::getNormalVector(
  hermite_path_msgs::msg::HermitePath path, double t)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = 3 * path.ay * t * t + 2 * path.by * t + path.cy;
  vec.y = (3 * path.ax * t * t + 2 * path.bx * t + path.cx) * -1;
  return vec;
}

visualization_msgs::msg::MarkerArray HermitePathGenerator::generateDeleteMarker()
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}

visualization_msgs::msg::MarkerArray HermitePathGenerator::generateMarker(
  std::vector<hermite_path_msgs::msg::HermitePathStamped> path, int resolution)
{
  visualization_msgs::msg::MarkerArray marker;
  std_msgs::msg::ColorRGBA color_center_line = color_names::makeColorMsg("green", 1.0);
  for (int i = 0; i < static_cast<int>(path.size()); i++) {
    hermite_path_msgs::msg::HermitePathStamped p = path[i];
    visualization_msgs::msg::Marker center_line;
    center_line.header = path[i].header;
    center_line.ns = "center_line";
    center_line.id = i;
    center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    center_line.action = visualization_msgs::msg::Marker::ADD;
    center_line.frame_locked = false;
    center_line.scale.x = 0.1;
    center_line.points = getPointsOnHermitePath(p.path, resolution, 1.0);
    center_line.colors =
      std::vector<std_msgs::msg::ColorRGBA>(center_line.points.size(), color_center_line);
    center_line.lifetime = rclcpp::Duration::from_seconds(3.0);
    marker.markers.push_back(center_line);
  }
  return marker;
}

visualization_msgs::msg::MarkerArray HermitePathGenerator::generateMarker(
  hermite_path_msgs::msg::HermitePathStamped path, int resolution, bool with_polygon)
{
  visualization_msgs::msg::MarkerArray marker;

  // Setup Color
  std_msgs::msg::ColorRGBA color_center_line = color_names::makeColorMsg("red", 1.0);
  // std_msgs::msg::ColorRGBA color_bounds = color_names::makeColorMsg("skyblue", 1.0);

  // Setup Center Line Marker
  visualization_msgs::msg::Marker center_line;
  center_line.header = path.header;
  center_line.ns = "center_line";
  center_line.id = 0;
  center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  center_line.action = visualization_msgs::msg::Marker::ADD;
  center_line.frame_locked = false;
  center_line.scale.x = 0.1;
  center_line.points = getPointsOnHermitePath(path.path, resolution, 1.0);
  center_line.colors =
    std::vector<std_msgs::msg::ColorRGBA>(center_line.points.size(), color_center_line);
  marker.markers.push_back(center_line);

  // Setup Left Bounds Marker
  if (with_polygon) {
    marker.markers.push_back(getBoundsPolygon(path, resolution, -0.3));
  }
  return marker;
}
}  // namespace hermite_path_planner
