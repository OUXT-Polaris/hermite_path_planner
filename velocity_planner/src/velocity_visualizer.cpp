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

#include <color_names/color_names.hpp>
#include <velocity_planner/velocity_visualizer.hpp>
#include <vector>
#include <string>

namespace velocity_planner
{
VelocityVisualizer::VelocityVisualizer(std::string node_name)
: generator_(0.0)
{
  node_name_ = node_name;
}

visualization_msgs::msg::MarkerArray VelocityVisualizer::generateDeleteMarker()
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}

visualization_msgs::msg::MarkerArray VelocityVisualizer::generateObstacleMarker(
  double t, hermite_path_msgs::msg::HermitePathStamped path, std_msgs::msg::ColorRGBA color,
  double width)
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker polygon;
  polygon.header = path.header;
  polygon.ns = "polygon";
  polygon.id = 0;
  polygon.type = polygon.TRIANGLE_LIST;
  polygon.action = polygon.ADD;
  polygon.scale.x = 1.0;
  polygon.scale.y = 1.0;
  polygon.scale.z = 1.0;
  geometry_msgs::msg::Vector3 normal = generator_.getNormalVector(path.path, t);
  double magnitude = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
  geometry_msgs::msg::Point point = generator_.getPointOnHermitePath(path.path, t);
  geometry_msgs::msg::Point p0l;
  p0l.x = point.x + width / 2.0 * normal.x / magnitude;
  p0l.y = point.y + width / 2.0 * normal.y / magnitude;
  p0l.z = 1.5;
  geometry_msgs::msg::Point p1l;
  p1l.x = point.x + width / 2.0 * normal.x / magnitude;
  p1l.y = point.y + width / 2.0 * normal.y / magnitude;
  p1l.z = 0.1;
  geometry_msgs::msg::Point p0r;
  p0r.x = point.x - width / 2.0 * normal.x / magnitude;
  p0r.y = point.y - width / 2.0 * normal.y / magnitude;
  p0r.z = 1.5;
  geometry_msgs::msg::Point p1r;
  p1r.x = point.x - width / 2.0 * normal.x / magnitude;
  p1r.y = point.y - width / 2.0 * normal.y / magnitude;
  p1r.z = 0.1;
  ret.markers.push_back(polygon);
  polygon.points.push_back(p0l);
  polygon.points.push_back(p0r);
  polygon.points.push_back(p1l);
  polygon.points.push_back(p1l);
  polygon.points.push_back(p0r);
  polygon.points.push_back(p1r);
  polygon.color = color;
  ret.markers.push_back(polygon);
  /*
            visualization_msgs::msg::Marker text;
            text.header = path.header;
            text.ns = "text";
            text.id = 0;
            text.type = text.TEXT_VIEW_FACING;
            text.action = text.ADD;
            text.scale.x = 1.0;
            text.scale.y = 1.0;
            text.scale.z = 1.0;
            */
  return ret;
}

double VelocityVisualizer::getVelocity(
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> vel, double t, double l)
{
  if (vel[0].t > t) {
    return vel[0].linear_velocity;
  }
  if (vel[vel.size() - 1].t < t) {
    return vel[vel.size() - 1].linear_velocity;
  }
  for (int i = 0; i < static_cast<int>(vel.size()); i++) {
    if (vel[i].t > t && vel[i + 1].t) {
      double dist = (vel[i + 1].t - vel[i].t) * l;
      double a = (vel[i + 1].linear_velocity * vel[i + 1].linear_velocity -
        vel[i].linear_velocity * vel[i].linear_velocity) /
        (2 * dist);
      if (vel[i].linear_velocity * vel[i].linear_velocity > 2 * dist * a) {
        return std::sqrt(vel[i].linear_velocity * vel[i].linear_velocity + 2 * dist * a);
      } else {
        return 0;
      }
    }
  }
  return 0;
}

visualization_msgs::msg::MarkerArray VelocityVisualizer::generatePolygonMarker(
  hermite_path_msgs::msg::HermitePathStamped path, double ratio, double width)
{
  constexpr int num_segment = 100;
  constexpr double segment_length = 1 / static_cast<double>(num_segment);
  visualization_msgs::msg::MarkerArray ret;
  for (int i = 0; i < num_segment; i++) {
    visualization_msgs::msg::Marker polygon;
    polygon.header = path.header;
    polygon.ns = "polygon";
    polygon.id = i;
    polygon.type = polygon.TRIANGLE_LIST;
    polygon.action = polygon.ADD;
    polygon.scale.x = 1.0;
    polygon.scale.y = 1.0;
    polygon.scale.z = 1.0;
    double t0 = static_cast<double>(i) * segment_length;
    double t1 = static_cast<double>(i + 1) * segment_length;
    double v0 = generator_.getReferenceVelocity(path, t0);
    double v1 = generator_.getReferenceVelocity(path, t1);
    double v = (v0 + v1) * 0.5;
    geometry_msgs::msg::Vector3 vec0 = generator_.getNormalVector(path.path, t0);
    geometry_msgs::msg::Vector3 vec1 = generator_.getNormalVector(path.path, t1);
    double theta0 = std::atan2(vec0.y, vec0.x);
    double theta1 = std::atan2(vec1.y, vec1.x);
    geometry_msgs::msg::Point p0 = generator_.getPointOnHermitePath(path.path, t0);
    geometry_msgs::msg::Point p1 = generator_.getPointOnHermitePath(path.path, t1);
    geometry_msgs::msg::Point p0l;
    p0l.x = p0.x - width * 0.5 * std::cos(theta0);
    p0l.y = p0.y - width * 0.5 * std::sin(theta0);
    p0l.z = v0 * ratio;
    geometry_msgs::msg::Point p0r;
    p0r.x = p0.x + width * 0.5 * std::cos(theta0);
    p0r.y = p0.y + width * 0.5 * std::sin(theta0);
    p0r.z = v0 * ratio;
    geometry_msgs::msg::Point p1l;
    p1l.x = p1.x - width * 0.5 * std::cos(theta1);
    p1l.y = p1.y - width * 0.5 * std::sin(theta1);
    p1l.z = v1 * ratio;
    geometry_msgs::msg::Point p1r;
    p1r.x = p1.x + width * 0.5 * std::cos(theta1);
    p1r.y = p1.y + width * 0.5 * std::sin(theta1);
    p1r.z = v1 * ratio;
    polygon.points.push_back(p0l);
    polygon.points.push_back(p0r);
    polygon.points.push_back(p1l);
    polygon.points.push_back(p1l);
    polygon.points.push_back(p0r);
    polygon.points.push_back(p1r);
    std_msgs::msg::ColorRGBA color;
    double ratio = std::fabs(v) / 0.8;
    polygon.color = color_names::fromHsv(0.6 * ratio, 1.0, 1.0, 1.0);
    ret.markers.push_back(polygon);
  }
  return ret;
}

visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
  hermite_path_msgs::msg::HermitePathStamped path)
{
  std_msgs::msg::ColorRGBA default_ref_color = color_names::makeColorMsg("lime", 1.0);
  return generateMarker(path, default_ref_color);
}

visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
  hermite_path_msgs::msg::HermitePathStamped path, std_msgs::msg::ColorRGBA color_ref_velocity)
{
  visualization_msgs::msg::MarkerArray marker;
  std::sort(
    path.reference_velocity.begin(), path.reference_velocity.end(),
    [](const auto & x, const auto & y) {return x.t < y.t;});
  for (auto itr = path.reference_velocity.begin(); itr != path.reference_velocity.end(); itr++) {
    visualization_msgs::msg::Marker box_marker;
    box_marker.header = path.header;
    box_marker.ns = "reference_velocity";
    box_marker.type = box_marker.CUBE;
    std::size_t index = std::distance(path.reference_velocity.begin(), itr);
    box_marker.id = index;
    geometry_msgs::msg::Point point = generator_.getPointOnHermitePath(path.path, itr->t);
    geometry_msgs::msg::Vector3 vec = generator_.getTangentVector(path.path, itr->t);
    double theta = std::atan2(vec.y, vec.x);
    geometry_msgs::msg::Vector3 orientation_vec;
    orientation_vec.x = 0.0;
    orientation_vec.y = 0.0;
    orientation_vec.z = theta;
    box_marker.action = box_marker.ADD;
    box_marker.pose.position = point;
    box_marker.pose.position.z = box_marker.pose.position.z + itr->linear_velocity * 0.5;
    box_marker.pose.orientation =
      quaternion_operation::convertEulerAngleToQuaternion(orientation_vec);
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    bool is_zero = (std::fabs(itr->linear_velocity) < DBL_EPSILON);
    if (is_zero) {
      box_marker.scale.z = 0.001;
    } else {
      box_marker.scale.z = itr->linear_velocity;
    }
    box_marker.color = color_ref_velocity;
    marker.markers.push_back(box_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = path.header;
    text_marker.ns = "reference_velocity_text";
    text_marker.type = text_marker.TEXT_VIEW_FACING;
    text_marker.action = text_marker.ADD;
    text_marker.id = index;
    text_marker.pose.position = point;
    text_marker.pose.position.z = text_marker.pose.position.z + itr->linear_velocity * 0.5 + 0.5;
    text_marker.scale.x = 0.2;
    text_marker.scale.y = 0.2;
    text_marker.scale.z = 0.2;
    text_marker.color = color_ref_velocity;
    char velocity_string[10];
    std::snprintf(velocity_string, sizeof(velocity_string), "%.2f", itr->linear_velocity);
    std::string str(velocity_string, 10);
    text_marker.text = str + "(m/s)";
    marker.markers.push_back(text_marker);
  }
  return marker;
}
}  // namespace velocity_planner
