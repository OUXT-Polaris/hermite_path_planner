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
#include <iostream>
#include <map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hermite_path_planner
{
class HermitePathGenerator
{
public:
  explicit HermitePathGenerator(double robot_width);
  hermite_path_msgs::msg::HermitePath generateHermitePath(
    const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal) const;
  geometry_msgs::msg::Point getPointOnHermitePath(
    const hermite_path_msgs::msg::HermitePath & path, double t) const;
  std::vector<geometry_msgs::msg::Point> getPointsOnHermitePath(
    const hermite_path_msgs::msg::HermitePath & path, int resolution, double max_t = 1.0) const;
  boost::optional<double> checkFirstCollisionWithCircle(
    hermite_path_msgs::msg::HermitePath path, geometry_msgs::msg::Point center,
    double radius) const;
  double getLength(const hermite_path_msgs::msg::HermitePath & path, int resolution) const;
  double getCurvature(const hermite_path_msgs::msg::HermitePath & path, double t) const;
  boost::optional<double> getNormalizedLongitudinalDistanceInFrenetCoordinate(
    const hermite_path_msgs::msg::HermitePath & path, const geometry_msgs::msg::Point & p) const;
  boost::optional<double> getLongitudinalDistanceInFrenetCoordinate(
    const hermite_path_msgs::msg::HermitePath & path, const geometry_msgs::msg::Point & p,
    int resolution) const;
  boost::optional<double> getLateralDistanceInFrenetCoordinate(
    const hermite_path_msgs::msg::HermitePath & path, const geometry_msgs::msg::Point & p) const;
  geometry_msgs::msg::Vector3 getTangentVector(
    const hermite_path_msgs::msg::HermitePath & path, double t) const;
  geometry_msgs::msg::Vector3 getNormalVector(
    const hermite_path_msgs::msg::HermitePath & path, double t) const;
  double getReferenceVelocity(hermite_path_msgs::msg::HermitePathStamped path, double t) const;
  visualization_msgs::msg::MarkerArray generateDeleteMarker() const;
  visualization_msgs::msg::MarkerArray generateMarker(
    const hermite_path_msgs::msg::HermitePathStamped & path, int resolution,
    bool with_polygon = true) const;
  visualization_msgs::msg::MarkerArray generateMarker(
    const std::vector<hermite_path_msgs::msg::HermitePathStamped> & path, int resolution) const;
  double getMaximumCurvature(
    const hermite_path_msgs::msg::HermitePath & path, int resolution) const;

private:
  hermite_path_msgs::msg::HermitePath generateHermitePath(
    const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal,
    double start_vector_magnitude, double end_vector_magnitude) const;
  visualization_msgs::msg::Marker getBoundsPolygon(
    const hermite_path_msgs::msg::HermitePathStamped & path, int resolution, double z_offset) const;
  geometry_msgs::msg::Vector3 getVectorFromPose(
    const geometry_msgs::msg::Pose & pose, double magnitude) const;
  std::vector<geometry_msgs::msg::Point> getLeftBounds(
    const hermite_path_msgs::msg::HermitePath & path, int resolution) const;
  std::vector<geometry_msgs::msg::Point> getRightBounds(
    const hermite_path_msgs::msg::HermitePath & path, int resolution) const;
  double robot_width_;
  double calculateNewtonMethodStepSize(
    const hermite_path_msgs::msg::HermitePath & path, const geometry_msgs::msg::Point & center,
    double radius, double t) const;
  inline double sigmoid(double gain, double x, double offset = 0.0) const
  {
    return 1.0 / (1.0 + exp(-gain * (x - offset)));
  }
};
}  // namespace hermite_path_planner

#endif  // HERMITE_PATH_PLANNER__HERMITE_PATH_GENERATOR_HPP_
