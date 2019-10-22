/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
class StaticTransformPublisher : public rclcpp::Node
{
public:
  explicit StaticTransformPublisher(
    double tx, double ty, double tz,
    double ax, double ay, double az, double aw,
    const char * frame_id, const char * child_id,
    rclcpp::NodeOptions options, const char * name);

  explicit StaticTransformPublisher(
    double tx, double ty, double tz,
    double r, double p, double y,
    const char * frame_id, const char * child_id,
    rclcpp::NodeOptions options, const char * name);

  explicit StaticTransformPublisher(
    const rclcpp::NodeOptions & options, const char * name = "StaticTransformPublisher");

  using TransformStamped = geometry_msgs::msg::TransformStamped;

  void send_transform(const TransformStamped & transform);

  ~StaticTransformPublisher() override = default;

private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_broadcaster;
  rclcpp::TimeSource m_time_source;
  std::shared_ptr<rclcpp::Clock> m_clock;
  TransformStamped m_tf_msg;
};
}
#endif  // TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_
