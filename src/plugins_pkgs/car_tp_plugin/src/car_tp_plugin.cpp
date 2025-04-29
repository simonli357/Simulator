#include "car_tp_plugin.hpp"
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gazebo {
namespace tp {

void TP::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr) {
  // Initialize ROS if not already
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "tp_plugin", ros::init_options::NoSigintHandler);
  }

  // Determine namespace from SDF
  std::string ns = "";
  if (sdf_ptr->HasElement("rosTopicNamespace")) {
    ns = sdf_ptr->Get<std::string>("rosTopicNamespace");
    if (!ns.empty() && ns.front() != '/')
      ns = "/" + ns;
  }

  // Create ROS node handle in that namespace
  m_ros_node = std::make_unique<ros::NodeHandle>(ns);

  // Subscribe to teleport topic
  std::string topic = ns + "/localisation/teleport";
  m_subTeleport = m_ros_node->subscribe<geometry_msgs::PoseStamped>(
      topic, 1, &TP::OnTeleport, this);
  ROS_INFO_STREAM("[tp_plugin] Subscribed to teleport: " << topic);

  // Store model pointer
  m_model = model_ptr;

  // Connect to Gazebo update event
  m_update_conn =
      event::Events::ConnectWorldUpdateBegin(std::bind(&TP::OnUpdate, this));
}

void TP::OnTeleport(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  // Convert PoseStamped to ignition::math::Pose3d using Vector3 + Quaternion
  // ctor
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.orientation, q);
  ignition::math::Quaterniond ig_q(q.getW(), q.getX(), q.getY(), q.getZ());
  ignition::math::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  m_target = ignition::math::Pose3d(pos, ig_q);
  m_have_target = true;
}

void TP::OnUpdate() {
  if (!m_have_target)
    return;
  // Teleport model instantly (non-blocking)
  m_model->SetWorldPose(m_target, /*auto-teleport-links=*/true);
}

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(TP)

} // namespace tp
} // namespace gazebo
