// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_
#define TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_

#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster.h"

#include "builtin_interfaces/msg/time.hpp"


namespace tf2_ros
{
class static_transform_component : public rclcpp::Node
{
public:
  explicit static_transform_component(
    double tx, double ty, double tz,
    double ax, double ay, double az, double aw,
    const char * frame_id, const char * child_id,
    rclcpp::NodeOptions options, const char * name);

  explicit static_transform_component(
    double tx, double ty, double tz,
    double r, double p, double y,
    const char * frame_id, const char * child_id,
    rclcpp::NodeOptions options, const char * name);

  explicit static_transform_component(
    const rclcpp::NodeOptions & options, const char * name = "static_transform_component");

  using TransformStamped = geometry_msgs::msg::TransformStamped;

  void send_transform(const TransformStamped & transform);

  ~static_transform_component() override = default;

private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_broadcaster;
  rclcpp::TimeSource m_time_source;
  std::shared_ptr<rclcpp::Clock> m_clock;
  TransformStamped m_tf_msg;
};
}
#endif  // TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_
