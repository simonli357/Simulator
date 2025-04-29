#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace gazebo {
namespace tp {

class TP : public ModelPlugin {
public:
  TP() = default;
  ~TP() override = default;

  // Gazebo calls this once on load
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // ROS callback for teleport commands
  void OnTeleport(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // Gazebo’s per-tick hook
  void OnUpdate();

private:
  // ROS side
  std::unique_ptr<ros::NodeHandle> m_ros_node;
  ros::Subscriber m_subTeleport;
  ignition::math::Pose3d m_target;
  bool m_have_target{false};

  // Gazebo side
  gazebo::physics::ModelPtr m_model;
  gazebo::event::ConnectionPtr m_update_conn;
};

} // namespace tp
} // namespace gazebo
