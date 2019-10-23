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
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_time_source.attachClock(m_clock);
  m_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (; !m_parameters_client->wait_for_service(1s); ) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "interrupted while waiting for parameter server, exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for parameters service...");
  }
  m_param_callback =
    m_parameters_client->on_parameter_event(this,
      std::bind(&StaticTransformPublisher::on_parameter_change, this, std::placeholders::_1));
}

void StaticTransformPublisher::on_parameter_change(const ParameterEvent event)
{
  auto map_parameter_to_tf = [this](const std::string & parameter_name, const double value) -> bool
    {
      // TODO(allenh1): Deal with remapping
      if (parameter_name == "/translation/x") {
        m_tf_msg.transform.translation.x = value;
        return true;
      } else if (parameter_name == "/translation/y") {
        m_tf_msg.transform.translation.y = value;
        return true;
      } else if (parameter_name == "/translation/z") {
        m_tf_msg.transform.translation.z = value;
        return true;
      } else if (parameter_name == "/rotation/x") {
        m_tf_msg.transform.rotation.x = value;
        return true;
      } else if (parameter_name == "/rotation/y") {
        m_tf_msg.transform.rotation.y = value;
        return true;
      } else if (parameter_name == "/rotation/z") {
        m_tf_msg.transform.rotation.z = value;
        return true;
      } else if (parameter_name == "/rotation/w") {
        m_tf_msg.transform.rotation.w = value;
        return true;
      }
      return false; // unknown message field
    };

  auto map_parameter_to_frame =
    [this](const std::string & parameter_name, const std::string & value) -> bool
    {
      if (parameter_name == "/frame_id") {
        if (m_tf_msg.child_frame_id == value) {
          RCLCPP_ERROR(this->get_logger(),
            "rejecting parameter '%s': child_frame_id cannot equal frame_id", parameter_name);
          return false;
        }
        m_tf_msg.header.frame_id = value;
        return true;
      } else if (parameter_name == "/child_frame_id") {
        if (m_tf_msg.header.frame_id == value) {
          RCLCPP_ERROR(this->get_logger(),
            "rejecting parameter '%s': child_frame_id cannot equal frame_id", parameter_name);
          return false;
        }
        m_tf_msg.child_frame_id = value;
        return true;
      }
      return false; // unknown message field
    };

  bool update = true;
  for (auto & new_parameter : event->new_parameters) {
    update &= (map_parameter_to_tf(new_parameter.name, new_parameter.value.double_value) ||
      map_parameter_to_frame(new_parameter.name, new_parameter.value.string_value));
  }

  for (auto & changed_parameter : event->changed_parameters) {
    update &= (map_parameter_to_tf(changed_parameter.name, changed_parameter.value.double_value) ||
      map_parameter_to_frame(changed_parameter.name, changed_parameter.value.string_value));
  }

  // check that the frame_id and child_frame_id are set
  if (update && !m_tf_msg.header.frame_id.empty() && !m_tf_msg.child_frame_id.empty()) {
    // update timestamp
    m_tf_msg.header.stamp = m_clock->now();
    // publish static transform
    RCLCPP_INFO(
      this->get_logger(),
      "publishing new transform from '%s' to '%s'", m_tf_msg.header.frame_id,
      m_tf_msg.child_frame_id);
    m_broadcaster->sendTransform(m_tf_msg);
  }
}
}  // namespace tf2_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_ros::StaticTransformPublisher)
