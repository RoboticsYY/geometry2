#ifndef TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_
#define TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_

#include <string_view>

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
  explicit static_transform_component(
    double tx, double ty, double tz,
    double ax, double ay, double az, double aw,
    std::string_view frame_id, std::string_view child_id,
    rclcpp::NodeOptions options, std::string_view name);

  explicit static_transform_component(
    double tx, double ty, double tz,
    double r, double p, double y,
    std::string_view frame_id, std::string_view child_id,
    rclcpp::NodeOptions options, std::string_view name);

  ~static_transform_component() override = default;

private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_broadcaster;
  rclcpp::TimeSource m_time_source;
  std::shared_ptr<rclcpp::Clock> m_clock;
  geometry_msgs::msg::TransformStamped m_tf_msg;
};
}
#endif  // TF2_ROS__STATIC_TRANSFORM_COMPONENT_HPP_
