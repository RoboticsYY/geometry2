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

#include "tf2_ros/static_transform_component.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace tf2_ros
{
StaticTransformPublisher::StaticTransformPublisher(
  rclcpp::NodeOptions options, const char * name
)
: rclcpp::Node(name, options)
{
  rclcpp::ParameterValue default_value(static_cast<double>(0));
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  auto tx = this->declare_parameter("/translation/x", default_value, descriptor);
  auto ty = this->declare_parameter("/translation/y", default_value, descriptor);
  auto tz = this->declare_parameter("/translation/z", default_value, descriptor);
  auto rx = this->declare_parameter("/rotation/x", default_value, descriptor);
  auto ry = this->declare_parameter("/rotation/y", default_value, descriptor);
  auto rz = this->declare_parameter("/rotation/z", default_value, descriptor);
  auto rw = this->declare_parameter("/rotation/w", default_value, descriptor);
  auto p_frame_id =
    this->declare_parameter("/frame_id", rclcpp::ParameterValue(std::string("/frame")), descriptor);
  auto p_child_id =
    this->declare_parameter("/child_frame_id", rclcpp::ParameterValue(std::string(
        "/child")), descriptor);

  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::TimeSource time_source;
  time_source.attachClock(clock);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.transform.translation.x = tx.get<double>();
  tf_msg.transform.translation.y = ty.get<double>();
  tf_msg.transform.translation.z = tz.get<double>();
  tf_msg.transform.rotation.x = rx.get<double>();
  tf_msg.transform.rotation.y = ry.get<double>();
  tf_msg.transform.rotation.z = rz.get<double>();
  tf_msg.transform.rotation.w = rw.get<double>();
  tf_msg.header.frame_id = p_frame_id.get<std::string>();
  tf_msg.child_frame_id = p_child_id.get<std::string>();
  // check frame_id != child_frame_id
  if (tf_msg.header.frame_id == tf_msg.child_frame_id) {
    RCLCPP_ERROR(this->get_logger(),
      "cannot publish static transform from '%s' to '%s', exiting",
      tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    rclcpp::shutdown();
    return;
  }
  // send transform
  tf_msg.header.stamp = clock->now();
  m_broadcaster->sendTransform(tf_msg);
}
}  // namespace tf2_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_ros::StaticTransformPublisher)
