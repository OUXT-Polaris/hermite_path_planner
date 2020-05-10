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

#ifndef HERMITE_PATH_PLANNER__HERMITE_PATH_GENERATOR_HPP_
#define HERMITE_PATH_PLANNER__HERMITE_PATH_GENERATOR_HPP_

#include <quaternion_operation/quaternion_operation.h>
#include <boost/optional.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <vector>
#include <map>

namespace hermite_path_planner
{
class HermitePathGenerator
{
public:
  explicit HermitePathGenerator(double robot_width);
  hermite_path_msgs::msg::HermitePath generateHermitePath(
    geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose goal);
  geometry_msgs::msg::Point getPointOnHermitePath(
    hermite_path_msgs::msg::HermitePath path, double t);
  std::vector<geometry_msgs::msg::Point> getPointsOnHermitePath(
    hermite_path_msgs::msg::HermitePath path, int resolution, double max_t = 1.0);
  boost::optional<double> checkFirstCollisionWithCircle(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point center, double radius);
  double getLength(hermite_path_msgs::msg::HermitePath path, int resolution);
  double getCurvature(hermite_path_msgs::msg::HermitePath path, double t);
  boost::optional<double> getNormalizedLongitudinalDistanceInFrenetCoordinate(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p);
  boost::optional<double> getLongitudinalDistanceInFrenetCoordinate(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p, int resolution);
  boost::optional<double> getLateralDistanceInFrenetCoordinate(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point p);
  geometry_msgs::msg::Vector3 getTangentVector(hermite_path_msgs::msg::HermitePath path, double t);
  geometry_msgs::msg::Vector3 getNormalVector(hermite_path_msgs::msg::HermitePath path, double t);
  double getReferenceVelocity(hermite_path_msgs::msg::HermitePathStamped path, double t);
  visualization_msgs::msg::MarkerArray generateDeleteMarker();
  visualization_msgs::msg::MarkerArray generateMarker(
    hermite_path_msgs::msg::HermitePathStamped path, int resolution, bool with_polygon = true);
  visualization_msgs::msg::MarkerArray generateMarker(
    std::vector<hermite_path_msgs::msg::HermitePathStamped> path, int resolution);
  double getMaximumCurvature(hermite_path_msgs::msg::HermitePath path, int resolution);

private:
  hermite_path_msgs::msg::HermitePath generateHermitePath(
    geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose goal,
    double start_vector_magnitude, double end_vector_magnitude);
  visualization_msgs::msg::Marker getBoundsPolygon(
    hermite_path_msgs::msg::HermitePathStamped path, int resolution, double z_offset);
  geometry_msgs::msg::Vector3 getVectorFromPose(geometry_msgs::msg::Pose pose, double magnitude);
  std::vector<geometry_msgs::msg::Point> getLeftBounds(
    hermite_path_msgs::msg::HermitePath path, int resolution);
  std::vector<geometry_msgs::msg::Point> getRightBounds(
    hermite_path_msgs::msg::HermitePath path, int resolution);
  double robot_width_;
  double calculateNewtonMethodStepSize(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point center, double radius,
    double t);
  inline double sigmoid(double gain, double x, double offset = 0.0)
  {
    return 1.0 / (1.0 + exp(-gain * (x - offset)));
  }
};
}  // namespace hermite_path_planner

#endif  // HERMITE_PATH_PLANNER__HERMITE_PATH_GENERATOR_HPP_
