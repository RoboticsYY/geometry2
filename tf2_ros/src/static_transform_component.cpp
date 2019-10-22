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


namespace tf2_ros
{
StaticTransformPublisher::StaticTransformPublisher(
  double tx, double ty, double tz,
  double ax, double ay, double az, double aw,
  const char * frame_id, const char * child_id,
  rclcpp::NodeOptions options, const char * name
)
: rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    std::make_shared<rclcpp::Node>(tf2_name, options));
  m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_time_source.attachClock(m_clock);

  m_tf_msg.transform.translation.x = tx;
  m_tf_msg.transform.translation.y = ty;
  m_tf_msg.transform.translation.z = tz;

  m_tf_msg.transform.rotation.x = ax;
  m_tf_msg.transform.rotation.y = ay;
  m_tf_msg.transform.rotation.z = az;
  m_tf_msg.transform.rotation.w = aw;

  m_tf_msg.header.stamp = m_clock->now();
  m_tf_msg.header.frame_id = frame_id;
  m_tf_msg.child_frame_id = child_id;

  m_broadcaster->sendTransform(m_tf_msg);

  RCLCPP_INFO(this->get_logger(), "Spinning until killed publishing %s to %s", frame_id, child_id);
}

StaticTransformPublisher::StaticTransformPublisher(
  double tx, double ty, double tz,
  double r, double p, double y,
  const char * frame_id, const char * child_id,
  rclcpp::NodeOptions options, const char * name
)
: rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    std::make_shared<rclcpp::Node>(tf2_name, options));
  m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_time_source.attachClock(m_clock);

  m_tf_msg.transform.translation.x = tx;
  m_tf_msg.transform.translation.y = ty;
  m_tf_msg.transform.translation.z = tz;

  tf2::Quaternion quat;
  quat.setRPY(r, p, y);
  m_tf_msg.transform.rotation.x = quat.x();
  m_tf_msg.transform.rotation.y = quat.y();
  m_tf_msg.transform.rotation.z = quat.z();
  m_tf_msg.transform.rotation.w = quat.w();

  m_tf_msg.header.stamp = m_clock->now();
  m_tf_msg.header.frame_id = frame_id;
  m_tf_msg.child_frame_id = child_id;

  RCLCPP_INFO(this->get_logger(), "Spinning until killed publishing %s to %s", frame_id, child_id);
}

StaticTransformPublisher::StaticTransformPublisher(
  const rclcpp::NodeOptions & options,
  const char * name)
: rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    std::make_shared<rclcpp::Node>(tf2_name, options));
}

void StaticTransformPublisher::send_transform(const TransformStamped & transform)
{
  m_broadcaster->sendTransform(transform);
}
}  // namespace tf2_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_ros::StaticTransformPublisher)
