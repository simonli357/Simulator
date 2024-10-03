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
            physics::ModelPtr                   m_model;
            ros::NodeHandlePtr		  nh;
            ros::Timer				  timer;

            /** ----------------------------------For ROS integration----------------------------------------------------**/
            // A node use for ROS transport
            std::unique_ptr<ros::NodeHandle>    m_ros_node;

            // A ROS publisher
            ros::Publisher                      m_pubBNO;
            ros::Publisher                      m_pubIMU;
            // ros::Publisher                      m_pubEncoder;
            sensor_msgs::Imu                    m_imu_msg;
            
            utils::encoder                      m_encoder_msg;
            utils::IMU                  m_bno055_pose;
            ignition::math::Vector3d prev_linear_velocity;
            ros::Time prev_time;

        // Default constructor
        public: BNO055();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};