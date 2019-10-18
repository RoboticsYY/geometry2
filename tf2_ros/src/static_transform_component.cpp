#include "tf2_ros/static_transform_component.hpp"
#include "rclcpp/rclcpp.hpp"


namespace tf2_ros
{

static_transform_component::static_transform_component(
  double tx, double ty, double tz,
  double ax, double ay, double az, double aw,
  const char * frame_id, const char * child_id,
  rclcpp::NodeOptions options, const char * name
) : rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(std::make_shared<rclcpp::Node>(tf2_name, options));
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

static_transform_component::static_transform_component(
  double tx, double ty, double tz,
  double r, double p, double y,
  const char * frame_id, const char * child_id,
  rclcpp::NodeOptions options, const char * name
) : rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(std::make_shared<rclcpp::Node>(tf2_name, options));
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

static_transform_component::static_transform_component(const rclcpp::NodeOptions & options, const char * name)
  : rclcpp::Node(name, options)
{
  std::string tf2_name{std::string(name) + "_tf2"};
  m_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(std::make_shared<rclcpp::Node>(tf2_name, options));
}

void static_transform_component::send_transform(const TransformStamped & transform)
{
  m_broadcaster->sendTransform(transform);
}
}  // namespace tf2_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_ros::static_transform_component)
