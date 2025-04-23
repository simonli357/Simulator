// bno055_plugin.hpp
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "utils/IMU.h"
#include "utils/encoder.h"
#include "sensor_msgs/Imu.h"

namespace gazebo
{
  namespace bno055
  {
    class BNO055: public ModelPlugin
    {
      private:
        physics::ModelPtr          m_model;
        ros::NodeHandlePtr         nh;
        ros::Timer                 timer;

        std::unique_ptr<ros::NodeHandle>  m_ros_node;
        ros::Publisher                    m_pubBNO;
        ros::Publisher                    m_pubIMU;
        ros::Publisher                    m_pubEncoder;

        sensor_msgs::Imu          m_imu_msg;
        utils::IMU                m_bno055_pose;
        utils::encoder            m_encoder_msg;

        ignition::math::Vector3d  prev_linear_velocity;
        ros::Time                 prev_time;

      public:
        BNO055();
        void Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr) override;
        void OnUpdate();
    };
  }
}
